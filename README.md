# Autoware Sensor Drivers for ROS2 Galactic

这个仓库包含了一系列为基于ROS2 Galactic部署Autoware准备的传感器驱动。支持的硬件包括RoboSense的RS16激光雷达，Fixposition的VRTK2，OAK的双目相机和Agilex的mini底盘。
![Example Image](image/未命名绘图.drawio-2.png)

## 内容

1. **Fixposition VRTK2驱动**：用于接收和处理Fixposition VRTK2的数据。使用方法请参考[这里](https://github.com/fixposition/fixposition_driver)。

2. **RoboSense RS16激光雷达驱动**：用于接收和处理RoboSense RS16激光雷达的数据。使用方法请参考[这里](https://github.com/RoboSense-LiDAR/rslidar_sdk)。

3. **RoboSense激光雷达转换包**：将RoboSense输出的Velodyne格式的激光雷达点云转换成去除NaN的形式，使得lio-sam建图不报错。

4. **odom_to_llh功能包**：将Fixposition的odom话题转换为llh话题，方便后续在SLAM中添加GPS信息做好基础。

## 安装

### 环境

ubuntu 20.04
ROS2 Galactic

### 安装步骤

1. 安装Fixposition VRTK2驱动和RoboSense RS16激光雷达驱动，请按照各个驱动的链接中的安装指南进行安装。

2. 安装RoboSense激光雷达转换包和odom_to_llh功能包，执行以下命令：

```bash
colcon build
source install/setup.bash
```

## 使用

### LIDAR:

```bash
python3 autoware_lidar_time.py
source install/setup.bash
ros2 launch rslidar_sdk start.py
ros2 run rs_to_velodyne rs_to_velodyne 
# ros2 run lidar_adaption lidar_adaption //no use
```

### CAMERA:

```bash
source install/setup.bash
ros2 launch depthai_examples mobile_publisher.launch.py camera_model:=OAK-D-LITE
ros2 launch depthai_examples rgb_publisher.launch.py camera_model:=OAK-D-LITE
```

### VRTK2:

在串行模式下启动节点，运行：

```bash
ros2 launch fixposition_driver_ros2 serial.launch
```

在TCP模式（Wi-Fi）下：

```bash
ros2 launch fixposition_driver_ros2 tcp.launch
```

在TCP模式（以太网）下：

```bash
ros2 launch fixposition_driver_ros2 tcp.launch
```

### Building:

```bash
colcon build --packages-skip fixposition_driver_ros1 fixposition_odometry_converter_ros1
```

### VRTK2 odom topic to llh

```bash
source install/setup.bash 
ros2 run odm_to_llh odm_to_llh 
```

### git:

```bash
cd src 
git add .
git commit -m " "
git push origin main
```

### topic:

| sensor | topic | frame_id |
| --- | --- | --- |
| lidar | /points | lidar_link |
| imu | /fixposition/corr_imu | imu_link |
| gps | /sensing/gnss/ublox/nav_sat_fix | gnss_link |
| camera | /color/image | camera4/camera_link |

### 如何配置你自己的传感器

#### VRTK2:

1. 将fixposition_driver_ros2/src/data_to_ros2.cpp中的33行与50行直接改为固定frame_id
2. 将fixposition_driver_ros2/src/fixpostion_driver_node.cpp中的45行与46行设置为对应的rostopic

#### camera:

deptahi_examples/ros2_src/rgb_publisher:49、52line

#### lidar:

rs_to_velodyne/src/rs_to_velodyne.cpp 97,231 line,configure your own topic and the frame id.


#过程中的问题

## 任务9 -- SLAM环境搭建和部署

### 进度要点
- 目前已经完成了lio—sam的部署和运行，完成了IMU和激光雷达的外参标定（见任务9中附件）

### 存在的问题
1. Lidar和IMU时间戳同步还未实现，由于Lidar仅支持PPS和GPS时间戳同步方法，所以仅采用这种方法，目前存在LIDAR无法获取到GPRMC信息，PPS信息正常，无法进行时间戳同步；
  - 解决方案：由于GPRMC电平未满足要求，张工已协助，预计本周硬件可以完备，硬件完备后再行确认LIdar和IMU时间戳同步情况。done(2024.06.26)
2. 激光雷达目前固定方式不是刚性连接，故标定结果存在误差。
  - 解决方案：原来板子设计有问题，缺乏安装位，张工订了新的板子回来 done(2024.06.28)
    - 重新进行imu和lidar的外参标定，标定corrimu和lidar以及raw imu和lidar，并进行对比（肖韧负责06.28）；
3. 因为天气目前在室内进行slam部署，lio-sam建图的点云密度较为稀疏， 可能特征不足，无法建图，预计天气好转出去在尝试一下（见任务10附件）workaround(2024.07.01)
  - 解决方案：已重新在虚拟机上部署lio-sam的ros1版本，天气好转后，先用IPC来录制rosbag，后用虚拟机上的lio-sam的ros1版本
    - 实测结果：lio-sam ros2版本无法实现建图，不管室外室内，均会出现features not enough警告。
    - lio-sam ros2版本还存在其他很多问题，前期使用集中在TF变换异常报错，ros topic无法配置，只能使用默认的ros topic。
    - 故首次录制户外城市道路环境下激光雷达数据和imu数据，尝试在虚拟机上使用lio-sam ros1版本进行建图。（06.28）
    - 16线激光雷达目前为止看满足要求，已经建图完成done(2024.07.01)

### 任务内容描述 (2024.07.02)
1. 进行建图工作和其他传感器融合处理的前提是需要做时间戳同步和内外参标定
  1. imu内参标定：
  2. 使用fixposition_ros2_driver的cor_imu数据进行标定，参考： https://blog.csdn.net/QLeelq/article/details/114652777
  3. imu_lidar标定：
  4. https://blog.csdn.net/weixin_45205745/article/details/129462125时间戳同步方案是使用VRTK2传输pps给rs16激光雷达，传输GPRMC消息给IPC，IPC将信息转发给rs16，通过ros话题可以看到lidar是否有时间戳；
2. 此项建图任务我们使用ros2的bag record记录在ros2环境下的rs16激光雷达和VRTK2的corrIMU，navstate数据，之后使用rosbag convert将ros2的rosbag转换为ros1，之后在ros1的环境下运行lio-sam实现建图，要注意的是话题需要使用rosbag自带的话题转换工具改变bag里的话题来实现话题符合lio-sam的参数。完成结构看任务9附件。

## 任务10 -- 地图的转换与绘制

### 进度要点
  - Autoware可以接受的地图形式为MGRS格式的，计划使用基于ros2的leo drive的UTM到MGRS点云转换器；
    - 问题：TIER IV 的矢量地图生成器工具需要UTM地图，然而UTM地图需要GPS信息的slam建图，幸运的是我们录取数据同时也记录了GPS信息，因此我们可以使用GPS信息来生成UTM地图。
  - 解决方案：我们计划使用leo drive的UTM到MGRS点云转换器来生成MGRS格式的地图，然后使用TIER IV的矢量地图生成器工具来生成UTM地图。完成后，我们将比较两种地图的差异，并根据实际需求选择最适合的地图格式。

### 存在的问题
1. leo drive的UTM到MGRS点云转换器尚未完成，我们正在等待其发布。
  - 解决方案：我们正在与leo drive的开发团队保持联系，以获取最新的更新信息。同时，我们也在寻找其他可能的解决方案，以防止leo drive的转换器无法按时发布。
2. TIER IV的矢量地图生成器工具需要UTM地图，但我们的slam建图系统目前只能生成MGRS地图。
  - 解决方案：我们计划修改我们的slam建图系统，使其能够生成UTM地图。这将需要一些时间，但我们相信这是值得的，因为UTM地图在许多应用中都有广泛的使用。

### 任务内容描述 (2024.07.03)
1. 我们的主要任务是生成两种格式的地图：MGRS和UTM。这需要我们修改我们的slam建图系统，以及使用leo drive的UTM到MGRS点云转换器和TIER IV的矢量地图生成器工具。
2. 我们还需要解决leo drive的UTM到MGRS点云转换器尚未发布的问题。我们正在与leo drive的开发团队保持联系，以获取最新的更新信息。同时，我们也在寻找其他可能的解决方案，以防止leo drive的转换器无法按时发布。
3. 我们的最终目标是生成一个可以在Autoware中使用的地图。这将需要我们对生成的地图进行详细的测试，以确保其在Autoware中的性能。

## 贡献

欢迎任何形式的贡献，包括但不限于问题报告，功能建议，代码提交等。

## 许可

请参考各个驱动的许可信息。

