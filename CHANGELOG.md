# CHANGELOG

以下是`cpp-dev`分支的变更日志

## TODO

- `track_perception`尚未封装完善，可以考虑`sim_perception`替换
- `rviz`可视化（可能）由于积分误差走偏（gazebo仿真没有偏），期望通过gazebo获取数据更新状态

## V0.5.1

fix(dev-cpp): 修复最后一个点不停止的问题

refactor(dev-cpp): 删去值传递订阅辅助模板函数

## V0.5.0

refactor(dev-cpp)：重构right_angle_planner、pure_pursuit_controller

- `pure_pursuit_controller`改善追踪点选择策略，完成注释
- `right_angle_planner`的路径点发布改善，今发布前方路径点，避免路径反向

## V0.4.2

feat(dev-cpp): 新增演示视频`演示.mp4`

## V0.4.1

feat(dev-cpp): 完善启动脚本以及注释

- 默认启动自带 RViz + Gazebo GUI

## V0.4.0

refactor(dev-cpp): 重构right_angle_planner、pure_pursuit_controller

- 独立`right_angle_planner`为子目录，拆分main函数和RightAnglePlanner类
- 独立`pure_pursuit_controller`为子目录，拆分main函数和PurePursuitController类
- 删去`right_angle_planner`的硬编码解析路线，车辆仅根据锥桶自动规划路线
- 重写`pure_pursuit_controller`策略为跟踪点策略

## V0.3.0

refactor(dev-cpp): 重构cone_mapper

- 独立`cone_mapper`为子目录，拆分main函数和ConeMapper类
- 删除`right_angle_stack_cpp`命名空间
- 删除`ConeDetections`消息类型的冗余链路，仅保留`Map`消息类型

## V0.2.0

refactor(dev-cpp): 重构localization_fusion

- 独立`localization_fusion`为子目录，拆分main函数和LocalizationFusion类
- 删去了硬编码的经纬度，改为`use_first_gps_as_origin`控制
- 补充`localization_fusion`的参数配置

## V0.1.0

feat(dev-cpp): 新增c++的right_angle_stack版本

- 重构 python 的 right_angle_stack 为 c++ 版本
- 删去了兼容旧版本 gazebo 的启动项