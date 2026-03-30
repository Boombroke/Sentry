// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <cmath>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  accumulated_count_(0),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  has_localized_(false)
{
  // Original parameters
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  // New configurable parameters
  this->declare_parameter("max_iterations", 20);
  this->declare_parameter("accumulated_count_threshold", 20);
  this->declare_parameter("min_range", 0.5);
  this->declare_parameter("min_inlier_ratio", 0.3);
  this->declare_parameter("max_fitness_error", 1.0);
  this->declare_parameter("enable_periodic_relocalization", false);
  this->declare_parameter("relocalization_interval", 30.0);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("accumulated_count_threshold", accumulated_count_threshold_);
  this->get_parameter("min_range", min_range_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_fitness_error", max_fitness_error_);
  this->get_parameter("enable_periodic_relocalization", enable_periodic_relocalization_);
  this->get_parameter("relocalization_interval", relocalization_interval_);

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters: max_iterations=%d, accumulated_threshold=%d, min_range=%.2f, "
    "min_inlier_ratio=%.2f, max_fitness_error=%.2f, periodic=%s, interval=%.1fs",
    max_iterations_, accumulated_count_threshold_, min_range_, min_inlier_ratio_,
    max_fitness_error_, enable_periodic_relocalization_ ? "true" : "false",
    relocalization_interval_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  RCLCPP_INFO(
    this->get_logger(), "Target map after downsampling: %zu points (leaf_size=%.3f)",
    target_->size(), global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Always update scan metadata (needed for initialPoseCallback)
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  // After initial localization, only accumulate if periodic relocalization is enabled
  if (has_localized_ && !enable_periodic_relocalization_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  // Filter out near-range points (self-reflections, noise)
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->reserve(scan->size());
  const double min_range_sq = min_range_ * min_range_;
  for (const auto & pt : scan->points) {
    double dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (dist_sq >= min_range_sq) {
      filtered->push_back(pt);
    }
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *accumulated_cloud_ += *filtered;
    accumulated_count_++;
  }

  // Initial localization: trigger after accumulating enough frames
  if (!has_localized_ && accumulated_count_ >= accumulated_count_threshold_) {
    RCLCPP_INFO(
      this->get_logger(), "Accumulated %d frames (%zu points), performing initial registration...",
      accumulated_count_, accumulated_cloud_->size());

    bool success = performRegistration(false);
    if (success) {
      has_localized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial localization succeeded.");

      // Start periodic relocalization timer if enabled
      if (enable_periodic_relocalization_) {
        periodic_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(relocalization_interval_ * 1000.0)),
          std::bind(&SmallGicpRelocalizationNode::periodicRegistrationCallback, this));
        RCLCPP_INFO(
          this->get_logger(), "Periodic relocalization enabled at %.1fs interval.",
          relocalization_interval_);
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Initial registration failed quality check. Will retry with more frames...");
      // Don't set has_localized_, allow more frames to accumulate and retry
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    accumulated_cloud_->clear();
    accumulated_count_ = 0;
  }
}

void SmallGicpRelocalizationNode::periodicRegistrationCallback()
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);

  if (accumulated_cloud_->empty() || accumulated_count_ < accumulated_count_threshold_ / 2) {
    RCLCPP_DEBUG(
      this->get_logger(), "Periodic reloc: insufficient points (%d frames, %zu points). Skipping.",
      accumulated_count_, accumulated_cloud_->size());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Periodic relocalization: %d frames (%zu points)",
    accumulated_count_, accumulated_cloud_->size());

  performRegistration(true);

  accumulated_cloud_->clear();
  accumulated_count_ = 0;
}

bool SmallGicpRelocalizationNode::performRegistration(bool is_periodic)
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return false;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "GICP input: source=%zu points, target=%zu points",
    source_->size(), target_->size());

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  // Extract transform for logging
  const Eigen::Vector3d t = result.T_target_source.translation();
  const Eigen::Vector3d rpy =
    result.T_target_source.rotation().eulerAngles(0, 1, 2);

  RCLCPP_INFO(
    this->get_logger(),
    "GICP result: converged=%d, iterations=%zu, num_inliers=%zu, error=%.6f",
    result.converged, result.iterations, result.num_inliers, result.error);
  RCLCPP_INFO(
    this->get_logger(),
    "GICP transform: t=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]",
    t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z());

  if (!result.converged) {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
    return false;
  }

  // Quality gate: check inlier ratio
  double inlier_ratio = static_cast<double>(result.num_inliers) / source_->size();
  RCLCPP_INFO(this->get_logger(), "GICP inlier_ratio=%.3f (threshold=%.3f)",
    inlier_ratio, min_inlier_ratio_);

  if (inlier_ratio < min_inlier_ratio_) {
    RCLCPP_WARN(
      this->get_logger(),
      "GICP quality check FAILED: inlier_ratio=%.3f < min_inlier_ratio=%.3f",
      inlier_ratio, min_inlier_ratio_);
    return false;
  }

  // Quality gate: check fitness error per inlier
  if (result.num_inliers > 0) {
    double fitness_error = result.error / static_cast<double>(result.num_inliers);
    RCLCPP_INFO(this->get_logger(), "GICP fitness_error=%.6f (threshold=%.6f)",
      fitness_error, max_fitness_error_);

    if (fitness_error > max_fitness_error_) {
      RCLCPP_WARN(
        this->get_logger(),
        "GICP quality check FAILED: fitness_error=%.6f > max_fitness_error=%.6f",
        fitness_error, max_fitness_error_);
      return false;
    }
  }

  // For periodic relocalization, also check that the correction is reasonable
  if (is_periodic) {
    Eigen::Vector3d delta_t =
      result.T_target_source.translation() - result_t_.translation();
    double delta_dist = delta_t.norm();
    // Reject periodic corrections larger than 2 meters (likely a bad match)
    if (delta_dist > 2.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Periodic reloc: correction too large (%.3f m). Rejecting.", delta_dist);
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Periodic reloc: accepted correction of %.3f m", delta_dist);
  }

  result_t_ = previous_result_t_ = result.T_target_source;
  return true;
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  // Constrain map->odom to 2D (x, y, yaw only).
  // GICP returns full 6DOF but for a ground robot the map->odom correction
  // should only contain planar components. Non-zero z/roll/pitch from GICP
  // cause the robot to appear below the map and point cloud angles to mismatch.
  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Matrix3d rotation = result_t_.rotation();
  double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = 0.0;

  // Yaw-only quaternion: [0, 0, sin(yaw/2), cos(yaw/2)]
  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = std::sin(yaw / 2.0);
  transform_stamped.transform.rotation.w = std::cos(yaw / 2.0);

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
