# Autoware Sensor Drivers for ROS2 Galactic

这个仓库包含了一系列为基于ROS2 Galactic部署Autoware准备的传感器驱动。支持的硬件包括RoboSense的RS16激光雷达，Fixposition的VRTK2，OAK的双目相机和Agilex的mini底盘。

## 内容

1. **Fixposition VRTK2驱动**：用于接收和处理Fixposition VRTK2的数据。使用方法请参考[这里](https://github.com/fixposition/fixposition_driver)。

2. **RoboSense RS16激光雷达驱动**：用于接收和处理RoboSense RS16激光雷达的数据。使用方法请参考[这里](https://github.com/RoboSense-LiDAR/rslidar_sdk)。

3. **RoboSense激光雷达转换包**：将RoboSense输出的Velodyne格式的激光雷达点云转换成去除NaN的形式，使得lio-sam建图不报错。

4. **odom_to_llh功能包**：将Fixposition的odom话题转换为llh话题，方便后续在SLAM中添加GPS信息做好基础。

## 安装

请按照各个驱动的链接中的安装指南进行安装。

## 使用

请按照各个驱动的链接中的使用指南进行使用。

## 贡献

欢迎任何形式的贡献，包括但不限于问题报告，功能建议，代码提交等。

## 许可

请参考各个驱动的许可信息。