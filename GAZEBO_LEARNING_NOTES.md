# Gazebo 仿真学习笔记

一开始安装 Gazebo 选错版本，装成 Classic 了。它和 Harmonic / Gazebo Sim 的语法、启动方式、ROS 通信包都有区别，起初还因为这个查了不少错误方向。后来才意识到，做 ROS 2 Jazzy 下的仿真，主线应该放在 Gazebo Sim 8 这套工具链上。

这次学 Gazebo 不是系统地从教程学起，而是工程哪里卡住就补哪里。现在回头看，真正要理解的是：Gazebo 负责仿真世界和传感器，ROS 负责算法节点，中间靠 bridge 把数据接起来。

## SDF 和 URDF 不要混

SDF 和 URDF/xacro 都可以用于描述车，给车建模，但使用场景不同。

Gazebo 真正加载、碰撞、运动、传感器插件这些，主要看 SDF。

RViz 和 `robot_state_publisher` 则主要看 URDF/xacro，因为 ROS 侧需要的是 link、joint 和 TF 关系，所以 Gazebo 里有车和 RViz 里有车不是完全等同。一个改 SDF，一个改 URDF，两个都要对。

world 文件负责更大的环境，比如重力、地面、光照、磁场、经纬度参考和赛道模型。

GPS 那里用到的 `spherical_coordinates` 也是在 world 里配置的。一开始我没重视这个，后面定位飞到几万米外时，才意识到 world 里的经纬度参考和 Gazebo NavSat 输出对不齐会直接影响定位。

我现在理解的层级大概是：

```text
world
  -> include track / car / cone models
model
  -> link
  -> joint
  -> visual / collision / inertial
  -> sensor / plugin
```

其中 `visual` 只管看起来是什么样，`collision` 才影响碰撞，`inertial` 会影响质量和运动稳定性。之前锥桶 mesh 找不到时，我一开始以为是模型没加载，其实只是 visual 的 mesh 路径有问题。

## link、joint、plugin 的理解

刚开始看车辆 SDF 时，`link`、`joint`、`plugin` 混在一起很乱。后来按功能拆开就清楚多了。

`link` 可以理解成车上的刚体，比如 `base_link`、车轮、相机支架、雷达支架。`joint` 用来描述两个 link 怎么连，比如车轮绕哪个轴转。`plugin` 则是 Gazebo 里的功能模块，比如 DiffDrive 插件负责把速度命令变成车轮运动。

这次小车用的是 DiffDrive 近似，所以控制输入是 `/cmd_vel`。它和真实赛车模型不完全一样，但直角弯任务里先跑通建图、规划、控制闭环更重要。以后如果要做得更像赛车，还需要更复杂的车辆动力学模型。

## 坐标系是最不能糊弄的地方

课程给的感知代码要求 `world` 和 `base_link` 这些名字，不能随便改。world 用 ENU，base_link 用 FLU。车从 `(0, -15)` 出发，朝北，所以 yaw 是 `pi/2`。

这个值如果错了，不只是车头方向不对，后面的感知范围、锥桶建图和 Pure Pursuit 目标点都会错。建图里要把 `base_link` 下的锥桶转到 `world`，本质上就是依赖当前车的位置和 yaw。

## 传感器学到的东西

这次建了相机、雷达、GPS、IMU、里程计和磁力计。相机和雷达目前主要用于 RViz 可视化，控制链路没有直接用它们。真正参与定位的是 GPS、IMU、轮速里程计，磁力计暂时保留接口但默认不强融合。

磁力计这里是一个教训。理论上它能给航向，但如果没有把坐标系、磁偏角和插件输出关系搞清楚，直接融合可能让 yaw 突然跳。为了先跑通闭环，配置里把 `mag_gain` 设成了 0。

GPS 也不是拿来就能用。经纬度要转成局部米制坐标，后来还遇到第一帧 GPS 和 world 原点不完全对齐的问题。最后做法是把第一帧 GPS 映射到初始位姿，后续 GPS 再做相对修正。

Gazebo 里的传感器也有几个容易忽略的点。第一是 `frame_id`，RViz 显示和后续算法都依赖它。第二是更新频率，太低会让控制反馈滞后，太高又会增加仿真负担。第三是噪声，完全没有噪声的仿真看起来更顺，但算法一旦依赖这种理想数据，就不太像真实系统。

相机和雷达还会触发渲染相关问题。在 WSL 上，GPU lidar 和 camera 比 GPS、IMU 这类非渲染传感器更容易崩，所以后面才分出了 headless 入口。

## 仿真时间和 /clock

ROS 2 节点在仿真里最好统一使用 sim time。Gazebo 会发布 `/clock`，节点参数里要开 `use_sim_time`。如果 `/clock` 没有数据，或者有些节点用系统时间、有些节点用仿真时间，TF 和 topic 看起来就会很乱。

这次调试时我养成了先看 `/clock` 的习惯：

```bash
ros2 topic echo /clock --once
```

只要这个都不通，就先不要急着查定位和控制。

## bridge 通信的方向

Gazebo Sim 的 topic 不会自动变成 ROS 2 topic，要靠 `ros_gz_bridge` 桥接通信。这次最关键的是 `/cmd_vel` 的方向：

```text
ROS -> Gazebo: /cmd_vel
Gazebo -> ROS: /clock, odom, gps, imu, magnetometer, camera, lidar
```

如果桥接方向反了，ROS 里发布命令看起来正常，但 Gazebo 里的车还是不会动。调车不转时，我后来常用这个命令绕过上层算法：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.4}}" -r 10
```

如果这个能让车转，说明 Gazebo 车辆模型和通信基本没问题，接下来再查规划和控制。

## 锥桶看不见不一定是模型错

Gazebo 找 mesh 依赖资源路径。锥桶显示不出来，或者只显示成圆柱时，很多时候是 `GZ_SIM_RESOURCE_PATH` 或 `model://...` 路径没配好。

这次最后的做法是：visual 用 `.dae` mesh，让 Gazebo 画面里看起来像锥桶；collision 用 cylinder，避免复杂 mesh 碰撞导致物理仿真不稳定。这个折中很实用。

资源路径相关的几个点也值得记：

- Gazebo Sim 常看 `GZ_SIM_RESOURCE_PATH`；
- SDF 里 `model://xxx/...` 需要能在资源路径下找到对应模型；
- `colcon build` 后还要确认模型和 mesh 被安装到了 `install/.../share/...`；
- 改了模型文件后，如果用的是 install 目录启动，最好重新 build/source。

这个问题表面上是“锥桶不显示”，实际经常是安装和路径问题。

## 物理仿真不要太理想化

Gazebo 里一个模型能显示，不代表它能稳定运动。车辆要动起来，质量、惯量、碰撞形状、关节轴、插件参数都要基本合理。

这次没有深入做车辆动力学，但至少学到两点：

- collision 不一定要和 visual 完全一样，简单稳定更重要；
- 惯量和质量如果乱写，车可能抖动、穿模或者控制响应很奇怪。

所以后面如果要继续完善车辆模型，不能只改外观，还要看 `inertial`、轮子半径、轮距、关节方向、摩擦和控制插件参数。

## WSL 能用，但不要太相信图形

WSL 上能验证很多东西，但相机、GPU 雷达、RViz、Gazebo GUI 这些可视化部分由于内存分配问题容易崩溃。遇到过 GUI 起不来、RViz 闪退、渲染线程崩溃，也遇到过在 `/mnt/e` 下面 build 时 CMake 写文件失败。

最后只在 WSL 里先跑 headless（这里是一开始担心ros2版本兼容问题就没到实体机上跑，后来发现多虑了）：

```bash
ros2 launch right_angle_stack right_angle_wsl_headless.launch.py use_rviz:=false
```

它不加载相机和 GPU 雷达，先验证 Gazebo server、spawn、bridge、定位、建图、规划和控制。headless 都没通时，不要急着查 RViz。

WSL 里还有一个经验：不要直接在 `/mnt/c`、`/mnt/e` 下面编译大工程。Windows 文件系统挂载进 WSL 后，权限和文件操作有时会出问题，CMake 生成文件时就可能失败。放到 WSL 自己的 ext4 文件系统里会稳很多。

## RViz 和 Gazebo 分工

一开始我会把 Gazebo 和 RViz 都当成“可视化界面”。后来发现它们看的东西不一样。

Gazebo 看的是仿真世界本身：车有没有 spawn、锥桶模型是否加载、传感器插件是否在世界里。RViz 看的是 ROS 数据：TF、LaserScan、PointCloud2、Image、MarkerArray、Path 等。

所以 RViz 报 `frame [world] does not exist` 时，不一定是 RViz 配置错了，可能是定位节点没发布 TF，也可能是 Gazebo 没启动导致 `/clock` 没有。Gazebo 里看不到锥桶，也不代表 ROS 侧没有锥桶 topic。两个界面要配合看。

## 我现在的调试顺序

先看基础：

```bash
ros2 topic echo /clock --once
ros2 node list
ros2 topic list
```

再看传感器：

```bash
ros2 topic echo /sensors/wheel_odom --once
ros2 topic echo /sensors/imu/data_raw --once
ros2 topic echo /sensors/gps/fix --once
ros2 topic echo /sensors/magnetic_field --once
```

再看算法链：

```bash
ros2 topic echo /localization/pose --once
ros2 topic echo /perception/cones --once
ros2 topic echo /estimation/slam/map --once
ros2 topic echo /planning/centerline --once
ros2 topic echo /cmd_vel --once
```

车不转时，先看 `/cmd_vel.angular.z`。如果角速度一直是 0，就查路径、定位和 lookahead 点；如果角速度不是 0，但 Gazebo 里不转，就查 DiffDrive 和 bridge。

这套顺序现在看起来很普通，但它确实比一开始“打开 Gazebo 盯着看”有效很多。仿真任务里最重要的是把问题定位到层级：是 Gazebo 模型层、bridge 层、定位层、感知建图层，还是控制层。

## 还没完全学透的地方

这次主要是为了把任务跑通，所以很多 Gazebo 细节只是学到了够用的程度。后面如果继续做，比较值得补的是：

- 更真实的 Ackermann 车辆模型；
- 轮胎摩擦和地面接触参数；
- 相机、雷达数据真正进入感知算法，而不是只在 RViz 里看；
- rosbag 录制仿真数据，方便复盘；
- Gazebo Sim 插件参数的系统整理。
