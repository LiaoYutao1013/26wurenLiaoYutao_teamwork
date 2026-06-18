# CHANGELOG

以下是`cpp-dev`分支的变更日志

## V0.2.0

refactor(dev-cpp): 重构localization_fusion

- 独立`localization_fusion`为子目录，拆分main函数和LocalizationFusion类
- 删去了硬编码的经纬度，改为`use_first_gps_as_origin`控制
- 补充`localization_fusion`的参数配置

## V0.1.0

feat(dev-cpp): 新增c++的right_angle_stack版本

- 重构 python 的 right_angle_stack 为 c++ 版本
- 删去了兼容旧版本 gazebo 的启动项