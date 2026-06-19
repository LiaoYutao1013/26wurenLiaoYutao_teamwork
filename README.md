# Gazebo 仿真任务：直角弯

## 工程结构

包功能：

- fsd_common_msgs：锥桶消息接口，包含 Cone、ConeDetections、Map。
- right_angle_track：赛道 world、锥桶模型、锥桶 mesh、赛道布置。
- right_angle_stack：车辆模型、launch、RViz、定位、建图、规划、控制。
- sim_perception：加密感知包。

以下是调试时要重点观察的话题：

| 层级 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 仿真控制输入 | /cmd_vel | geometry_msgs/msg/Twist | 控制器输出，桥接到 Gazebo DiffDrive |
| 轮速里程计 | /sensors/wheel_odom | nav_msgs/msg/Odometry | Gazebo 车辆运动反馈 |
| GPS | /sensors/gps/fix | sensor_msgs/msg/NavSatFix | 经纬度定位输入 |
| IMU | /sensors/imu/data_raw | sensor_msgs/msg/Imu | 角速度、线加速度 |
| 磁力计 | /sensors/magnetic_field | sensor_msgs/msg/MagneticField | 航向辅助，当前默认不强融合 |
| 相机 | /sensors/camera/image_raw | sensor_msgs/msg/Image | RViz 可视化 |
| 相机内参 | /sensors/camera/camera_info | sensor_msgs/msg/CameraInfo | RViz/Image 工具使用 |
| 雷达扫描 | /sensors/lidar/scan | sensor_msgs/msg/LaserScan | RViz 可视化 |
| 雷达点云 | /sensors/lidar/scan/points | sensor_msgs/msg/PointCloud2 | RViz 可视化 |
| 定位输出 | /localization/pose | geometry_msgs/msg/PoseStamped | 建图和感知使用 |
| 定位里程计 | /localization/odom | nav_msgs/msg/Odometry | 控制器使用 |
| 感知锥桶 | /perception/cones | fsd_common_msgs/msg/Map | base_link 下的局部锥桶 |
| 感知检测 | /perception/cone_detections | fsd_common_msgs/msg/ConeDetections | 单帧锥桶检测 |
| 建图输出 | /estimation/slam/map | fsd_common_msgs/msg/Map | world 下的锥桶地图 |
| 规划路径 | /planning/centerline | nav_msgs/msg/Path | 控制器跟踪路径 |
| 建图可视化 | /visualization/cone_map | visualization_msgs/msg/MarkerArray | RViz 锥桶 |
| 规划可视化 | /visualization/planning | visualization_msgs/msg/MarkerArray | RViz 中心线 |

## 运行前

### 清理旧进程

```bash
pkill -INT -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
sleep 2
pkill -9 -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
```

### 终端环境

```bash
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
```

### 构建

切换到工程根目录下后，

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+ 
source install/setup.bash
```

### 完整启动

切换到工程根目录下后，

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf" 
```

## 协作记录

任务大致拆分成以下部分：

- 环境：ROS/Gazebo 安装、WSL 图形栈、构建流程。
- 仿真：world、车辆、传感器、bridge、RViz。
- 定位：GPS/IMU/轮速/磁力计融合和 TF。
- 感知建图：锥桶观测、坐标变换、地图融合。
- 规划控制：中心线、Pure Pursuit、速度和角速度输出。
- 文档：记录问题、验证命令、解决方法、截图。

### 踩坑记录

- WSL 上容易遇到 Gazebo 或 RViz 启动后闪退，尤其是相机、GPU 雷达这类会触发渲染线程的传感器。后面保留了 right_angle_wsl_headless.launch.py，专门用来先验证非渲染链路。
- Jazzy 对接口文件检查更严格，最开始 .msg 走 CMake 生成失败，后来把接口改成显式 .idl 后通过。
- Gazebo Classic 和 Gazebo Sim 的启动方式、模型格式、ROS bridge 都不完全一样。这个版本最后按 Gazebo Harmonic 整理，Classic 入口只当兼容文件保留。
- Gazebo 对资源路径比较敏感，路径里有特殊字符时排错会麻烦，所以最后把工程目录整理成 percep_node_track 这种更普通的命名(在最终提交的作业仓库里直接展开了下属的文件夹，命名同样遵循这个原则)。
- fsd_common_msgs 在 Jazzy 由于无法正常生成.idl目标文件编译失败，结合AI分析推测是因为Jazzy 比 Humble 语法检查更严格。现在保留 .msg，但 CMake 实际使用AI转换的相应 .idl 构建。（这部分编码问题暂时没找到更好的解决方案）
- 定位大幅偏离预期，如有一次 /localization/pose 返回值x,y高达五位数（？）推测GPS 经纬度原点和 Gazebo NavSat 输出没对齐，或者磁力计航向突变。最后通过第一帧 GPS 映射到初始位姿，GPS 突变拒绝，磁力计额外添加固定默认值0，标定后再启用等方法解决。
- 加密包经常丢失，导致运行时感知失灵，目前只能尽量做到协作开发代码时保持环境一致，并写好 .gitignore,防止误屏蔽毁掉整个工程😡

## AI使用情况

AI主要用于排错工具和文档阅读。日志很长时，先让 AI 帮忙把错误分层，比如区分是 CMake 接口生成失败、Gazebo 资源路径问题，还是渲染线程崩溃。后面再根据它给的排查命令自己去验证。
AI 参与过 launch、SDF、RViz、定位融合和 README 的整理，也辅助写了内置track节点作为调试时的节点，和 sim_perception 相对独立。
