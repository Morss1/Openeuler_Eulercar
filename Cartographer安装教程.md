# Cartographer建图过程介绍

这里只给出源码安装方式，若简单apt安装请百度搜索、

## 1.源码安装

```sh
创建工作间
mkdir ros2_ws
cd ros2_ws
mkdir src
cd src

```

将下面的源码克隆到ros2_ws的src目录下

```sh
git clone https://github.com/ros2/cartographer.git -b ros2
git clone https://github.com/ros2/cartographer_ros.git -b ros2
```

## 2.依赖安装

运行如下命令进行依赖的安装

```sh
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

> 如果依赖安装失败或没有rosdepc，使用如下命令先安装rosdepc

```sh
wget http://fishros.com/install -O fishros && . fishros
```

选择一键配置rosdep即可

> rosdepc 是小鱼制作的国内版rosdep，是一个用于安装依赖的工具。该工具的安装可以采用[一键安装](https://fishros.org.cn/forum/topic/20)进行，选项编号为3。安装完成后运行一次rodepc update即可使用。

## 3.编译

```sh
colcon build --packages-up-to cartographer_ros
```

> –packages-up-to，意思是其所有依赖后再编译该包

## 4.测试安装是否成功

```sh
ros2 pkg list | grep cartographer
```

能看到下面结果即可

```sh
cartographer_ros
cartographer_ros_msgs
```

## 5.Cartographer参数配置

### 5.1前端参数

#### 文件：trajectory_builder_2d

路径：src/cartographer/configuration_files/trajectory_builder_2d.lua

```lua
  -- 是否使用IMU数据
  use_imu_data = true, 
  -- 深度数据最小范围
  min_range = 0.,
  -- 深度数据最大范围
  max_range = 30.,
  -- 传感器数据超出有效范围最大值时，按此值来处理
  missing_data_ray_length = 5.,
  -- 是否使用实时回环检测来进行前端的扫描匹配
  use_online_correlative_scan_matching = true
  -- 运动过滤，检测运动变化，避免机器人静止时插入数据
  motion_filter.max_angle_radians
```

### 5.2后端参数

#### 文件：pose_graph.lua

路径：src/cartographer/configuration_files/pose_graph.lua

```lua
--Fast csm的最低分数，高于此分数才进行优化。
constraint_builder.min_score = 0.65
--全局定位最小分数，低于此分数则认为目前全局定位不准确
constraint_builder.global_localization_min_score = 0.7
```

### 5.3Carotgrapher_ROS参数配置

#### 文件：backpack_2d.lua

路径：src/cartographer_ros/cartographer_ros/configuration_files/backpack_2d.lua

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- 用来发布子地图的ROS坐标系ID，位姿的父坐标系，通常是map。
  map_frame = "map",
  -- SLAM算法跟随的坐标系ID
  tracking_frame = "base_link",
  -- 将发布map到published_frame之间的tf
  published_frame = "base_link",
  -- 位于“published_frame ”和“map_frame”之间，用来发布本地SLAM结果（非闭环），通常是“odom”
  odom_frame = "odom",
  -- 是否提供里程计
  provide_odom_frame = true,
  -- 只发布二维位姿态（不包含俯仰角）
  publish_frame_projected_to_2d = false,
  -- 是否使用里程计数据
  use_odometry = false,
  -- 是否使用GPS定位
  use_nav_sat = false,
  -- 是否使用路标
  use_landmarks = false,
  -- 订阅的laser scan topics的个数
  num_laser_scans = 0,
  -- 订阅多回波技术laser scan topics的个数
  num_multi_echo_laser_scans = 1,
  -- 分割雷达数据的个数
  num_subdivisions_per_laser_scan = 10,
  -- 订阅的点云topics的个数
  num_point_clouds = 0,
  -- 使用tf2查找变换的超时秒数
  lookup_transform_timeout_sec = 0.2,
  -- 发布submap的周期间隔
  submap_publish_period_sec = 0.3,
  -- 发布姿态的周期间隔
  pose_publish_period_sec = 5e-3,
  -- 轨迹发布周期间隔
  trajectory_publish_period_sec = 30e-3,
  -- 测距仪的采样率
  rangefinder_sampling_ratio = 1.,
  --里程记数据采样率
  odometry_sampling_ratio = 1.,
  -- 固定的frame位姿采样率
  fixed_frame_pose_sampling_ratio = 1.,
  -- IMU数据采样率
  imu_sampling_ratio = 1.,
  -- 路标采样率
  landmarks_sampling_ratio = 1.,
}
```

参考文章：

https://blog.csdn.net/qq_27865227/article/details/132525447?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522172481287416800225526597%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fblog.%2522%257D&request_id=172481287416800225526597&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~blog~first_rank_ecpm_v1~rank_v31_ecpm-2-132525447-null-null.nonecase&utm_term=Cartographer&spm=1018.2226.3001.4450