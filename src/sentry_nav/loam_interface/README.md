# loam_interface

## 简介
LOAM里程计接口封装包,负责将point_lio输出的lidar_odom坐标系转换为Nav2使用的odom坐标系。这是坐标系转换链 lidar_odom -> odom -> chassis 的关键节点。

## 功能
- 坐标系转换: 将lidar_odom系下的里程计和点云变换到odom系。
- 静态变换应用: 通过TF查找base_frame->lidar_frame静态变换。
- 节点模式: 使用Composable Node模式,支持高效集成。

## 话题
### 订阅
- `state_estimation_topic` (nav_msgs/Odometry): point_lio输出的原始里程计。
- `registered_scan_topic` (sensor_msgs/PointCloud2): 注册后的点云数据。

### 发布
- `lidar_odometry` (nav_msgs/Odometry): 在odom frame下的里程计。
- `registered_scan` (sensor_msgs/PointCloud2): 在odom frame下的点云。

## 参数
- `state_estimation_topic`: 订阅的里程计话题名称。
- `registered_scan_topic`: 订阅的点云话题名称。
- `odom_frame`: 目标里程计坐标系,默认"odom"。
- `base_frame`: 机器人底盘坐标系。
- `lidar_frame`: 激光雷达坐标系。

## 使用
该节点通常作为导航启动链的一部分,确保里程计数据符合Nav2的标准坐标系要求。
