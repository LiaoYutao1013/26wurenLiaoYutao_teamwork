# CHANGELOG

以下是`cpp-dev`分支的变更日志

## TODO

- `right_angle_planner`的追踪点随着地图更新而更新，但是还没过滤好已经过的轨迹点（健壮性不好，但是样例看不出来）
- `pure_pursuit_controller`注释亟待完成
- `track_perception`尚未封装完善，可以考虑`sim_perception`替换
- `rviz`可视化走偏（gazebo仿真没有偏），期望通过gazebo获取数据更新状态

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