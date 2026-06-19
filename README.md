# Gazebo 仿真任务：直角弯

调试过程中在WSL和实体机上都试运行过，小组开发环境有Humble和Jazzy（但组员多数用jazzy，只在WSL上启用过Humble），也绕过 Gazebo Classic 和 Gazebo Sim 两条路线。
最后项目在实体机运行最容易成功，WSL 的图形渲染不稳定，只适合开始时调试数据或检查与图形渲染无关节点的通信情况。

一些踩坑记录：

- WSL 上容易遇到 Gazebo 或 RViz 启动后闪退，尤其是相机、GPU 雷达这类会触发渲染线程的传感器。后面保留了 right_angle_wsl_headless.launch.py，专门用来先验证非渲染链路。
- Jazzy 对接口文件检查更严格，最开始 .msg 走 CMake 生成失败，后来把接口改成显式 .idl 后通过。
- Gazebo Classic 和 Gazebo Sim 的启动方式、模型格式、ROS bridge 都不完全一样。这个版本最后按 Gazebo Harmonic 整理，Classic 入口只当兼容文件保留。
- Gazebo 对资源路径比较敏感，路径里有特殊字符时排错会麻烦，所以最后把工程目录整理成 percep_node_track 这种更普通的命名(在最终提交的作业仓库里直接展开了下属的文件夹，命名同样遵循这个原则)。
- fsd_common_msgs 在 Jazzy 由于无法正常生成.idl目标文件编译失败，结合AI分析推测是因为Jazzy 比 Humble 语法检查更严格。现在保留 .msg，但 CMake 实际使用AI转换的相应 .idl 构建。（这部分编码问题暂时没找到更好的解决方案）
- 定位大幅偏离预期，如有一次 /localization/pose 返回值x,y高达五位数（？）推测GPS 经纬度原点和 Gazebo NavSat 输出没对齐，或者磁力计航向突变。最后通过第一帧 GPS 映射到初始位姿，GPS 突变拒绝，磁力计额外添加固定默认值0，标定后再启用等方法解决。

## 坐标系约定

按照给出感知代码对坐标系的要求，对坐标系做如下约定：

- 全局坐标系：world
- 全局坐标系方向：ENU，x 向东，y 向北，z 向上
- 车辆底盘坐标系：base_link
- 车辆局部坐标系方向：FLU，x 向前，y 向左，z 向上
- 相机光学坐标系：camera_optical_frame
- 雷达坐标系：lidar_link
- 定位 TF：world -> base_link

车辆从 (0, -15) 出发，朝北行驶。world 的 y 轴向北，所以车辆初始 yaw 是 $\frac{\pi}{2}$。

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

## 运行前准备

### 清理旧进程

Gazebo 和 bridge 没退干净时，下一次 launch 可能会连到旧进程，导致误以为问题没解决。遇到这种情况先kill掉所有Gazebo有关的进程，或者直接重启。
（可能是电脑某些配置的问题，但这里还没研究清楚）

```bash
pkill -INT -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
sleep 2
pkill -9 -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
```

另外遇到过launch后手动打开Gazebo时,点击空项目却打开正在运行的地图这一问题。推断为启动命令打开了server，而Gazebo此时只是连接这个server的客户端。

### 终端环境准备

不要在同一个终端里混用不同开发环境（如Python版本）多个工作空间。之前因此导致编译和运行的python版本不匹配而报错。（所以虚拟环境的设置和隔离环境真的很重要）

```bash
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
```

### 工作区位置

如果在 WSL 里编译，建议把工程复制到 WSL 文件系统里，不要直接在挂载的windows分区下 build。之前在 Windows 盘路径下遇到过 CMake 写文件权限问题。实体机上没有这个问题。

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

### WSL 下分步调试

WSL 上图形栈不稳定时，先不要急着开 RViz。可以只启动 Gazebo server 和算法链路，先看 topic 是否正常发布。注意传感器带噪声，数值在小范围内波动是正常的。

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

如果一开相机或 GPU 雷达就崩溃退出，先用 headless 入口验证非渲染链路：

```bash
ros2 launch right_angle_stack right_angle_wsl_headless.launch.py use_rviz:=false
```

这个入口禁用相机和 GPU 雷达，只保留定位传感器、算法链路和控制闭环。它适合排查构建、spawn、bridge、定位、建图、规划和控制，不适合验证相机/雷达画面。

### 感知

默认启动使用 sim_perception

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

等价于默认参数

```bash
use_builtin_perception:=false
use_sim_perception:=true
```

如果加密包运行环境临时有问题，可以切回内置 track_perception 调试，看建图、规划、控制链路有没有问题。

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_builtin_perception:=true \
  use_sim_perception:=false \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

插曲：组员反馈sim_perception 报错缺少 pyarmor_runtime.so，发现它被之前编写的 .gitignore 屏蔽，导致组员使用同步后的代码测试时失败。

## 测试思路

### 仿真

先确认Gazebo 里车和锥桶渲染显示是否正常；ROS 侧传感器通信是否正常，车是否能动起来（期望是一开始运行就要前进）。

主要文件：

- tracks/worlds/right_angle_harmonic.sdf
- tracks/models/shixi/model.sdf
- tracks/models/blue_cone/model.sdf
- tracks/models/yellow_cone/model.sdf
- right_angle_stack/models/right_angle_car_harmonic/model.sdf
- right_angle_stack/urdf/right_angle_car.urdf.xacro
- right_angle_stack/launch/right_angle_harmonic.launch.py

right_angle_harmonic.sdf 里定义了 right_angle_world、重力、磁场、地面、光照、spherical_coordinates 和赛道模型。spherical_coordinates 主要给 GPS 插件提供经纬度参考。
根据AI对项目结构的分析，赛道本体在 shixi/model.sdf，通过 `<include>` 布置蓝锥和黄锥。直道沿 y 轴从 y=-15 到 y=0，随后是一个右角弯。

锥桶模型最后采用了一个折中做法：

- visual 使用 .dae mesh，Gazebo 里看起来是锥桶。
- collision 使用 cylinder，减少电脑负担（没显卡是这样的），避免 mesh collision 在物理引擎里引入不稳定。

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

Gazebo GUI 观察仿真场景，RViz 检查 ROS 侧数据（TF、车辆模型、相机图像、雷达点云、锥桶地图、规划中心线等）。

### 定位

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
短时间运动主要靠轮速和 IMU 推算：

- /sensors/wheel_odom.twist.twist.linear.x 作为车体前向速度；
- /sensors/imu/data_raw.angular_velocity.z 积分更新 yaw；
- GPS 慢速修正 x/y，超过 gps_reject_distance 的跳变会拒绝。

### 感知

最终启动默认使用 sim_perception/sim_node.py。项目里保留的 track_perception.py 是调试节点，用静态锥桶位置和车辆位姿模拟局部感知，方便在加密包或运行时库出问题时继续查建图、规划和控制。

### 建图

节点：right_angle_stack/right_angle_stack/cone_mapper.py
建图节点看输入/localization/pose，/perception/cones，/perception/cone_detections
输出给后面的规划/estimation/slam/map和 RViz /visualization/cone_map
感知消息一般在 base_link 下，建图要把它转回 world。如果输入 frame 已经是 world 或 map，就直接使用；否则根据 /localization/pose 做一次平面坐标变换：
$$
world_x = car_x + cos(yaw) * local_x - sin(yaw) * local_y \\
world_y = car_y + sin(yaw) * local_x + cos(yaw) * local_y
$$
锥桶按 blue/yellow/red/unknown 分 bucket 保存。对同色锥桶，如果新观测和已有 landmark 距离小于 merge_distance，就认为是同一个锥桶，用递推平均更新位置；否则创建新 landmark。地图发布为 fsd_common_msgs/msg/Map，frame 是 world。
RViz 中的 /visualization/cone_map 用 cylinder marker 显示，方便看建图结果。这里的 cylinder 只影响 RViz，不影响 Gazebo 中真实锥桶 mesh。
开发中遇到的关键问题是定位飞走会直接导致建图为空。因为建图依赖 /localization/pose 把局部锥桶转换到 world，定位的精度影响感知范围和地图。

### 规划

节点：right_angle_stack/right_angle_stack/right_angle_planner.py
订阅/estimation/slam/map，/localization/pose
发布/planning/centerline，/visualization/planning
规划优先用锥桶地图，如果地图暂时不够完整，就回退到解析路径，这样调试时不至于因为感知或建图一处没通就完全看不到运行效果。

锥桶地图中心线的构建思路如下：

1. 遍历蓝锥，找最近的黄锥配对；
2. 配对距离小于 pair_distance_max 才接受；
3. 取蓝黄锥中点作为中心线点；
4. 用 track_progress() 按赛道前进方向排序；
5. 用 densify() 加密路径点，减少控制器目标点跳变。

路径输出为 nav_msgs/msg/Path，frame 为 world。RViz 中绿色线是cone map产生的，白色路径是fallback。

### 控制

节点：right_angle_stack/right_angle_stack/pure_pursuit_controller.py
订阅/planning/centerline，/localization/odom
发布/cmd_vel
控制器用 Pure Pursuit。每次控制周期先找离车最近的路径点，再往后找一个在车前方、距离超过 lookahead_distance 的目标点。目标点转到 base_link 后，计算曲率$$curvature = 2 \frac{local_y}{lookahead^2}$$
角速度$$yaw_{rate} = target_{speed} * curvature + yaw_{error-gain} * yaw_{error}$$
最后限制 max_yaw_rate，并根据曲率做简单降速。弯道中速度降低，直道恢复目标速度。
如果 /cmd_vel.angular.z 已经非零但 Gazebo 里车不转，问题通常在 Gazebo 车辆模型或 /cmd_vel bridge；如果 /cmd_vel.angular.z 一直为零，问题通常在规划路径、车辆定位或 lookahead 目标点选择。

### 联调顺序

联调时没有一开始就盯着控制器调。先确认 Gazebo 能正常加载不崩溃，车、锥桶、传感器正常渲染，再确认车辆能够运动。之后逐步检查传感器、定位、感知、建图和规划。

组内主要观察了以下指标：

1. /clock、GPS、IMU、轮速、磁力计先有数据；
2. /localization/pose 留在赛道附近，不飞到几万米外；
3. /perception/cones 能看到车前方锥桶；
4. /estimation/slam/map 里有 world 下的锥桶；
5. /planning/centerline 有完整右角弯路径；
6. /cmd_vel 同时有速度和角速度；
7. Gazebo 里车能沿直道进入弯道，RViz 里能看到 TF、相机、雷达、锥桶地图和规划线。

## 调试时常用命令

### 先确认节点和时钟

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

### 车不转时

```bash
ros2 topic echo /localization/pose --once
ros2 topic echo /perception/cones --once
ros2 topic echo /estimation/slam/map --once
ros2 topic echo /planning/centerline --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /sensors/wheel_odom --once
```

通过以上命令返回的数据判断车的哪些节点出问题，当然部分数据（如位姿）也可以通过Gazebo查看。

- /localization/pose 离赛道很远，先查定位；
- /perception/cones 为空，先查感知范围、车辆位姿和锥桶模型；
- /estimation/slam/map 为空，多半是建图没收到锥桶，或者定位不可用；
- /planning/centerline 为空，看地图够不够，fallback 有没有关；
- /cmd_vel.angular.z 一直为 0，看规划路径和 lookahead 目标点；
- /cmd_vel.angular.z 非 0 但车不转，再去查 DiffDrive 和 bridge。

## 协作记录

任务大致拆分成以下部分：

- 环境：ROS/Gazebo 安装、WSL 图形栈、构建流程。
- 仿真：world、车辆、传感器、bridge、RViz。
- 定位：GPS/IMU/轮速/磁力计融合和 TF。
- 感知建图：锥桶观测、坐标变换、地图融合。
- 规划控制：中心线、Pure Pursuit、速度和角速度输出。
- 文档：记录问题、验证命令、解决方法、截图。

## AI使用情况

AI主要用于排错工具和文档阅读。日志很长时，先让 AI 帮忙把错误分层，比如区分是 CMake 接口生成失败、Gazebo 资源路径问题，还是渲染线程崩溃。后面再根据它给的排查命令自己去验证。
AI 参与过 launch、SDF、RViz、定位融合和 README 的整理，也辅助写了内置track节点作为调试时的节点，和 sim_perception 相对独立。
