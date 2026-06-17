# Gazebo 仿真任务：直角弯

这个工程是课程里的直角弯仿真任务，目标是把 Gazebo 里的车辆、传感器和 ROS 2 算法链路跑成一个闭环：

```text
Gazebo 赛道/车辆/传感器
  -> 定位融合
  -> 锥桶感知输入
  -> 建图
  -> 中心线规划
  -> Pure Pursuit 控制
  -> /cmd_vel
  -> Gazebo 车辆运动
```

目前稳定跑通的环境：

- ROS 2 Jazzy
- Gazebo Sim 8
- ros_gz_sim + ros_gz_bridge
- 工作区：homework/percep_node_track
- 起点 (0, -15)，车辆朝北，初始 yaw 为 $\frac{\pi}{2}$

调试过程中试过 WSL、实体机、Humble、Jazzy，也绕过 Gazebo Classic 和 Gazebo Sim 两条路线。最后的结论比较朴素：这个项目在实体机 + Jazzy + Gazebo Sim 8 上最省事，WSL 可以用来做 headless 验证，但图形和渲染传感器不太稳定。

几条实际踩坑记录：

- WSL 上容易遇到 Gazebo 或 RViz 启动后闪退，尤其是相机、GPU 雷达这类会触发渲染线程的传感器。后面保留了 right_angle_wsl_headless.launch.py，专门用来先验证非渲染链路。
- Jazzy 对接口文件检查更严格，最开始 .msg 走 CMake 生成失败，后来把接口改成显式 .idl 后才通过。
- Gazebo Classic 和 Gazebo Sim 的启动方式、模型格式、ROS bridge 都不完全一样。这个版本最后按 Gazebo Sim 8 整理，Classic 入口只当兼容文件保留。
- Gazebo 对资源路径比较敏感，路径里有特殊字符时排错会麻烦，所以最后把工程目录整理成 percep_node_track 这种更普通的命名。

## 坐标系约定

课程给出的感知代码对坐标系有要求，本工程按下面约定实现：

- 全局坐标系：world
- 全局坐标系方向：ENU，x 向东，y 向北，z 向上
- 车辆底盘坐标系：base_link
- 车辆局部坐标系方向：FLU，x 向前，y 向左，z 向上
- 相机光学坐标系：camera_optical_frame
- 雷达坐标系：lidar_link
- 定位 TF：world -> base_link

车辆从 (0, -15) 出发，朝北行驶。因为 world 的 y 轴向北，所以车辆初始 yaw 是 $\frac{\pi}{2}$。

## 工程结构

```text
percep_node_track/
├── fsd_common_msgs/
│   └── msg/
│       ├── Cone.idl
│       ├── ConeDetections.idl
│       └── Map.idl
├── right_angle_stack/
│   ├── config/right_angle_stack.yaml
│   ├── launch/
│   │   ├── right_angle_harmonic.launch.py
│   │   ├── right_angle_wsl_headless.launch.py
│   │   └── right_angle_sim.launch.py
│   ├── models/
│   │   ├── right_angle_car_harmonic/model.sdf
│   │   └── right_angle_car_wsl_headless/model.sdf
│   ├── right_angle_stack/
│   │   ├── localization_fusion.py
│   │   ├── track_perception.py
│   │   ├── cone_mapper.py
│   │   ├── right_angle_planner.py
│   │   ├── pure_pursuit_controller.py
│   │   ├── sim_sensor_bridge.py
│   │   ├── track_model.py
│   │   └── utils.py
│   ├── rviz/right_angle.rviz
│   └── urdf/right_angle_car.urdf.xacro
├── sim_perception/
│   └── sim_perception/
│       ├── sim_node.py
│       └── pyarmor_runtime_000000/
└── tracks/
    ├── models/
    │   ├── blue_cone/
    │   ├── yellow_cone/
    │   └── shixi/
    └── worlds/
        ├── right_angle_harmonic.sdf
        └── right_angle_wsl_headless.sdf
```

包功能：

- fsd_common_msgs：锥桶消息接口，包含 Cone、ConeDetections、Map。
- right_angle_track：赛道 world、锥桶模型、锥桶 mesh、赛道布置。
- right_angle_stack：车辆模型、launch、RViz、定位、建图、规划、控制。
- sim_perception：老师提供的加密感知包，保留原样。

## 系统数据流

调试时最常看的话题：

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

总启动命令默认使用老师给的 sim_perception 作为感知子系统。相机和雷达已经建模并桥接到 ROS 侧，主要用于 RViz 可视化，不参与控制决策。

## 环境与构建

### 清理旧进程

Gazebo 和 bridge 没退干净时，下一次 launch 可能会连到旧进程，现象看起来就像“代码没更新”。遇到这种情况先杀一遍进程，或者直接重启也行。

```bash
pkill -INT -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
sleep 2
pkill -9 -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
```

这个问题在实体机和 WSL 都遇到过，尤其是上一次 Gazebo GUI 没正常退出时。

### 终端环境

不要在同一个终端里混用 Humble、Jazzy 或多个工作空间。调试时最好新开终端，先清掉旧环境变量，再 source Jazzy：

```bash
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
```

### 工作区位置

如果在 WSL 里编译，建议把工程放到 WSL 自己的 ext4 文件系统里，不要直接在 /mnt/c 或 /mnt/e 下 build。之前在 Windows 盘路径下遇到过 CMake 写文件权限问题。实体机上基本不用管这个。

### 构建

```bash
cd ~/文档/SCUT_Racing_Tasks/homework/percep_node_track
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+ 
source install/setup.bash
```

## 启动流程

### 完整仿真：Gazebo GUI + RViz + 全部传感器

```bash
cd ~/文档/SCUT_Racing_Tasks/homework/percep_node_track
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf" 
```

### WSL 或分步调试

WSL 上图形栈不稳定时，先不要急着开 RViz。可以只启动 Gazebo server 和算法链路，先看 topic 是否正常发布。注意传感器带噪声，数值有小幅波动是正常的。

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=false \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

确认话题有数据后，再单独开 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source ~/文档/SCUT_Racing_Tasks/homework/percep_node_track/install/setup.bash

rviz2 -d ~/文档/SCUT_Racing_Tasks/homework/percep_node_track/install/right_angle_stack/share/right_angle_stack/rviz/right_angle.rviz
```

如果一开相机或 GPU 雷达就崩，先用 headless 入口验证非渲染链路：

```bash
ros2 launch right_angle_stack right_angle_wsl_headless.launch.py use_rviz:=false
```

这个入口禁用相机和 GPU 雷达，只保留定位传感器、算法链路和控制闭环。它适合排查构建、spawn、bridge、定位、建图、规划和控制，不适合验证相机/雷达画面。

### 感知算法默认设置

默认启动时已经使用老师给的 sim_perception：

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

等价于默认参数：

```bash
use_builtin_perception:=false
use_sim_perception:=true
```

如果加密包运行环境临时有问题，可以切回内置 track_perception 做 fallback 调试。这个节点不是比赛/最终感知方案，只是为了看清楚建图、规划、控制链路有没有问题。

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_builtin_perception:=true \
  use_sim_perception:=false \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

若 sim_perception 报缺少 pyarmor_runtime.so，检查

```bash
find sim_perception install/sim_perception -name 'pyarmor_runtime.so' -ls
```

该 .so 是运行加密包必需文件，不能被 .gitignore 屏蔽（先前误屏蔽了此文件，导致组员使用同步后的代码测试时失败）。

## 各环节开发

### 1. 仿真

仿真层先解决三个问题：Gazebo 里要有车和赛道，ROS 侧要能收到传感器，/cmd_vel 要能真正让车动起来。这个部分没稳定之前，后面的算法调参基本没有意义。

主要文件：

- tracks/worlds/right_angle_harmonic.sdf
- tracks/models/shixi/model.sdf
- tracks/models/blue_cone/model.sdf
- tracks/models/yellow_cone/model.sdf
- right_angle_stack/models/right_angle_car_harmonic/model.sdf
- right_angle_stack/urdf/right_angle_car.urdf.xacro
- right_angle_stack/launch/right_angle_harmonic.launch.py

right_angle_harmonic.sdf 里定义了 right_angle_world、重力、磁场、地面、光照、spherical_coordinates 和赛道模型。spherical_coordinates 主要给 GPS 插件提供经纬度参考。赛道本体在 shixi/model.sdf，通过 <include> 布置蓝锥和黄锥。直道沿 y 轴从 y=-15 到 y=0，随后接一个右角弯。

锥桶模型最后采用了一个折中做法：

- visual 使用 .dae mesh，Gazebo 里看起来是锥桶。
- collision 使用 cylinder，避免 mesh collision 在物理引擎里引入不稳定。

车辆模型 right_angle_car_harmonic/model.sdf 包含：

   - base_link
   - 四个车轮 link 和 revolute joint
   - DiffDrive 插件
   - 相机 link 和 camera sensor
   - 雷达 link 和 gpu_lidar sensor
   - GPS、IMU、磁力计 sensor

车辆通过 DiffDrive 接收 /cmd_vel，并发布 /sensors/wheel_odom。right_angle_harmonic.launch.py 中使用 ros_gz_bridge parameter_bridge 桥接：

   - ROS -> Gazebo：/cmd_vel
   - Gazebo -> ROS：/clock、轮速里程计、GPS、IMU、磁力计、相机、雷达

Gazebo GUI 主要看真实仿真场景，RViz 主要看 ROS 侧数据：TF、车辆模型、相机图像、雷达点云、锥桶地图和规划中心线。

### 2. 定位

定位节点：right_angle_stack/right_angle_stack/localization_fusion.py

订阅的传感器：

- /sensors/gps/fix
- /sensors/imu/data_raw
- /sensors/wheel_odom
- /sensors/magnetic_field

发布：

```text
/localization/pose
/localization/odom
TF：world -> base_link
```

GPS 经纬度先用局部切平面近似转成米制坐标：

```text
x = R * cos(lat0) * (lon - lon0)
y = R * (lat - lat0)
R = 6378137.0 m
```

这里有一个调试中才发现的坑：Gazebo NavSat 插件输出的经纬度和 world 里写死的参考点不一定完全对齐。直接按固定经纬度换算时，车的位置曾经被拉到几万米外。现在的做法是把第一帧 GPS 映射到初始位姿 (0, -15)，后续 GPS 只作为相对位置修正。

短时间运动主要靠轮速和 IMU 推：

- /sensors/wheel_odom.twist.twist.linear.x 作为车体前向速度；
- /sensors/imu/data_raw.angular_velocity.z 积分更新 yaw；
- GPS 慢速修正 x/y，超过 gps_reject_distance 的跳变会拒绝。

磁力计代码保留了，但配置里 mag_gain: 0.0。原因是未标定磁力计时航向容易突变，先不让它影响闭环；后面如果要认真融合，需要先把磁力计坐标系、磁偏角和 yaw 定义对清楚。

### 3. 感知输入

最终启动默认使用老师给的 sim_perception/sim_node.py。项目里保留的 track_perception.py 是调试兜底节点，用静态锥桶位置和车辆位姿模拟局部感知，方便在加密包或运行时库出问题时继续查建图、规划和控制。

### 4. 建图

节点：right_angle_stack/right_angle_stack/cone_mapper.py

建图节点主要看这些输入：

- /localization/pose
- /perception/cones
- /perception/cone_detections

输出给后面的规划和 RViz：

- /estimation/slam/map
- /visualization/cone_map

感知消息一般在 base_link 下，建图要把它转回 world。如果输入 frame 已经是 world 或 map，就直接使用；否则根据 /localization/pose 做一次平面坐标变换：

```text
world_x = car_x + cos(yaw) * local_x - sin(yaw) * local_y
world_y = car_y + sin(yaw) * local_x + cos(yaw) * local_y
```

锥桶按 blue/yellow/red/unknown 分 bucket 保存。对同色锥桶，如果新观测和已有 landmark 距离小于 merge_distance，就认为是同一个锥桶，用递推平均更新位置；否则创建新 landmark。地图发布为 fsd_common_msgs/msg/Map，frame 是 world。

RViz 中的 /visualization/cone_map 用 cylinder marker 显示，方便看建图结果。这里的 cylinder 只影响 RViz，不影响 Gazebo 中真实锥桶 mesh。

开发中遇到的关键问题是定位飞走会直接导致建图为空。因为建图依赖 /localization/pose 把局部锥桶转换到 world，所以一旦定位不可信，感知范围和地图都会错。

### 5. 规划开发

节点：right_angle_stack/right_angle_stack/right_angle_planner.py

订阅：

- /estimation/slam/map
- /localization/pose

发布：

- /planning/centerline
- /visualization/planning

规划器保留了两套路径来源。优先用锥桶地图，如果地图暂时不够完整，就回退到解析路径，这样调试时不至于因为感知或建图一处没通就完全看不到车运动。

锥桶地图中心线的做法比较直接：

- 遍历蓝锥，找最近的黄锥配对；
- 配对距离小于 pair_distance_max 才接受；
- 取蓝黄锥中点作为中心线点；
- 用 track_progress() 按赛道前进方向排序；
- 用 densify() 加密路径点，减少控制器目标点跳变。

解析 fallback 路径按直角弯几何生成：

- 第一段：沿 x=0 从 y=-15 走到 y=0；
- 第二段：以 (12, 0) 为圆心、半径 12 生成四分之一圆弯道；
- 第三段：沿 y=12 向东直行。

路径输出为 nav_msgs/msg/Path，frame 为 world。RViz 中规划线颜色用于区分来源：

- cone map 路径：偏绿色；
- fallback 路径：白色。

调试时如果看到车一开始直行，这是正常现象，因为起点到弯道入口本来就是直道。只有接近 y=0 后才应该开始右转。

### 6. 控制开发

节点：right_angle_stack/right_angle_stack/pure_pursuit_controller.py

订阅：

- /planning/centerline
- /localization/odom

发布：

- /cmd_vel

控制器用 Pure Pursuit。每次控制周期先找离车最近的路径点，再往后找一个在车前方、距离超过 lookahead_distance 的目标点。目标点转到 base_link 后，用下面的公式算曲率：

```text
curvature = 2 * local_y / lookahead^2
```

角速度使用：

```text
yaw_rate = target_speed * curvature + yaw_error_gain * yaw_error
```

最后限制 max_yaw_rate，并根据曲率做简单降速。弯道中速度降低，直道恢复目标速度。

如果 /cmd_vel.angular.z 已经非零但 Gazebo 里车不转，问题通常在 Gazebo 车辆模型或 /cmd_vel bridge；如果 /cmd_vel.angular.z 一直为零，问题通常在规划路径、车辆定位或 lookahead 目标点选择。

### 7. 闭环联调过程

联调时按下面顺序推进：

1. Gazebo 能启动，车辆和锥桶可见。
2. /cmd_vel 能通过 bridge 让 Gazebo 车辆运动。
3. GPS、IMU、轮速、磁力计 topic 有数据。
4. /localization/pose 在赛道附近，不飞到几万米外。
5. /perception/cones 在车辆前方有锥桶。
6. /estimation/slam/map 有 world 下的锥桶。
7. /planning/centerline 有完整右角弯路径。
8. /cmd_vel 有速度和角速度。
9. Gazebo 中车辆沿直道进入右角弯。
10. RViz 中能看到 TF、车辆模型、相机、雷达、锥桶地图和规划线。

## 调试流程

### 总体检查

```bash
ros2 node list
ros2 topic list
ros2 topic echo /clock --once
```

正常情况下能看到这些节点：

```text
/ros_gz_bridge
/robot_state_publisher
/localization_fusion
/sim_perception
/cone_mapper
/right_angle_planner
/pure_pursuit_controller
```

只有手动切换到 fallback 感知时，才会额外出现 /track_perception。

### 传感器检查

```bash
ros2 topic echo /sensors/wheel_odom --once
ros2 topic echo /sensors/imu/data_raw --once
ros2 topic echo /sensors/gps/fix --once
ros2 topic echo /sensors/magnetic_field --once
```

相机和雷达：

```bash
gz topic -l | grep -E 'camera|lidar|sensors'

ros2 topic list | grep -E 'camera|lidar'
ros2 topic echo /sensors/camera/camera_info --once
ros2 topic hz /sensors/camera/image_raw
ros2 topic echo /sensors/lidar/scan --once
ros2 topic echo /sensors/lidar/scan/points --once
```

### 不转向排查

```bash
ros2 topic echo /localization/pose --once
ros2 topic echo /perception/cones --once
ros2 topic echo /estimation/slam/map --once
ros2 topic echo /planning/centerline --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /sensors/wheel_odom --once
```

排查时按这个方向看：

- /localization/pose 坐标离赛道很远：定位融合问题。
- /perception/cones 为空：感知范围、车辆位姿或锥桶 SDF 读取问题。
- /estimation/slam/map 为空：建图没有收到局部锥桶或定位不可用。
- /planning/centerline 为空：规划器没有地图，或者 fallback 被关闭。
- /cmd_vel.angular.z 一直为 0：优先检查规划路径、车辆定位和 lookahead 目标点。
- /cmd_vel.angular.z 非 0 但车不转：优先检查 Gazebo DiffDrive 和 bridge。

### 直接测试车辆驱动

绕过规划控制，直接发速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.4}}" -r 10
```

如果车能动并转向，车辆模型和 bridge 基本正常，问题在上游算法链路。

## 常见问题复盘

### fsd_common_msgs 在 Jazzy 下编译失败

现象：

```text
rosidl_generate_interfaces.cmake: list index: 1 out of range
Target dependency ... Cone.idl does not exist
```

这个问题最后定位到接口生成链路。Jazzy 比 Humble 更严格，原来的 .msg 适配成 .idl 时失败。现在保留 .msg 方便阅读，但 CMake 实际使用对应 .idl 构建。

### CMakeCache 路径不一致

现象：

```text
The current CMakeCache.txt directory ... is different than the directory ...
```

这是因为把已经构建过的 workspace 从 Windows 路径复制到 WSL 路径时，build/ 里还记录着旧的绝对路径。删掉构建产物后重新编译：

```bash
rm -rf build install log
colcon build --symlink-install
```

### Gazebo 找不到模型 mesh

现象：

```text
Unable to find file ... cone_blue.dae
Failed to load geometry for visual
```

- 确认 tracks/models/blue_cone/meshes/cone_blue.dae 存在。
- 确认 launch 设置了 GZ_SIM_RESOURCE_PATH。
- 锥桶 visual 使用 model://blue_cone/meshes/cone_blue.dae。
- collision 继续使用 cylinder。

### RViz 报 frame [world] does not exist

通常不是 RViz 的问题，而是 Gazebo 或定位节点没有正常跑起来。先检查：

```bash
ros2 topic echo /clock --once
ros2 topic echo /tf --once
ros2 topic echo /localization/pose --once
```

### 定位飞到几万米外

现象：

```text
/localization/pose:
x: -16866
y: 12862
```

当时主要怀疑两个方向：GPS 经纬度原点和 Gazebo NavSat 输出没有对齐，或者磁力计航向突变参与融合。最后采用的处理：

- 第一帧 GPS 映射到初始位姿；
- GPS 突变拒绝；
- 磁力计默认 mag_gain: 0.0，后续标定后再启用。

## 组内协作

后续如果继续分工，比较适合按链路切：

- 环境：ROS/Gazebo 安装、WSL 图形栈、构建流程。
- 仿真：world、车辆、传感器、bridge、RViz。
- 定位：GPS/IMU/轮速/磁力计融合和 TF。
- 感知建图：锥桶观测、坐标变换、地图融合。
- 规划控制：中心线、Pure Pursuit、速度和角速度输出。
- 文档：记录问题、验证命令、解决方法、截图。

讨论问题时尽量按数据链路往下查，不要直接猜是哪一块代码写错。例如“不转向”先看：

text
cmd_vel 是否有角速度？
planning 是否有弯道？
localization 是否在赛道附近？
perception 是否看到锥桶？
Gazebo 是否收到 /cmd_vel？


这样比只盯 Gazebo 画面快很多，也方便不同成员各自负责一段 topic。

## AI 使用情况

AI 主要用在排错和整理资料上，尤其是日志很长、错误层级不明显的时候。具体用过的地方：

- 阅读长日志，归类错误层级；
- 生成排查命令；
- 辅助修改 launch、SDF、RViz 和定位融合代码；
- 实现内置备份感知节点，和老师给的 sim_perception 区分开，只用于调试；
- 整理 README。

几个关键决策还是需要自己能解释清楚，比如为什么 GPS 第一帧要映射到初始位姿、为什么磁力计默认不参与融合、为什么锥桶 collision 不直接用 mesh、为什么 WSL 下要有 headless 入口。答辩时这些不能只说“AI 这样写的”。

## 答辩准备

答辩前建议至少能解释下面这些：

- 为什么 world 用 ENU，base_link 用 FLU？
- 为什么起点 (0, -15) 朝北对应 yaw pi/2？
- GPS 经纬度如何转局部米制坐标？
- 为什么第一帧 GPS 被用作仿真局部参考？
- 为什么磁力计当前默认不强融合？
- 为什么锥桶 visual 用 mesh，collision 用 cylinder？
- 为什么相机和雷达当前只用于可视化？
- 建图如何把 base_link 下锥桶变成 world 下地图？
- 规划如何从蓝黄锥生成中心线？
- fallback 路径为什么前半段是直行？
- Pure Pursuit 如何根据目标点算角速度？
- 如果车不转，按什么顺序排查？
