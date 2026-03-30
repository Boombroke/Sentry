# sensor_scan_generation

## 简介
传感器扫描生成节点,负责同步里程计和点云数据,并输出chassis系下的统一数据流。

## 功能
- 时间同步: 使用message_filters对lidar_odometry和registered_scan进行精确同步。
- 坐标变换: 将数据转换至chassis系。
- TF广播: 负责广播 odom -> chassis 的TF变换。
- 节点模式: 使用Composable Node模式。

## 话题
### 订阅
- `lidar_odometry` (nav_msgs/Odometry): odom系下的里程计。
- `registered_scan` (sensor_msgs/PointCloud2): odom系下的点云。

### 发布
- `odometry` (nav_msgs/Odometry): chassis系下的里程计。
- `sensor_scan` (sensor_msgs/PointCloud2): chassis系下的点云。

## 参数
- `lidar_frame`: 激光雷达坐标系。
- `base_frame`: 机器人基础坐标系。
- `robot_base_frame`: 机器人底盘坐标系。

## 使用
该节点为后续的地形分析和局部规划提供同步且坐标统一的传感器数据。
