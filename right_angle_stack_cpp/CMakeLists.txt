cmake_minimum_required(VERSION 3.8)
project(right_angle_stack_cpp)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fsd_common_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# localization
add_subdirectory(src/localization)
# cone_mapper
add_subdirectory(src/cone_mapper)
# right_angle_planner
add_subdirectory(src/right_angle_planner)
# pure_pursuit_controller
add_subdirectory(src/pure_pursuit_controller)

add_executable(track_perception
        src/track_perception.cpp
        src/track_model.cpp)
ament_target_dependencies(track_perception
        rclcpp
        fsd_common_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
        visualization_msgs
)

install(TARGETS
        localization_fusion
        track_perception
        cone_mapper
        right_angle_planner
        pure_pursuit_controller
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch config models rviz urdf
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()