add_executable(right_angle_planner
        right_angle_planner.cpp
        main.cpp
)
ament_target_dependencies(right_angle_planner
        rclcpp
        fsd_common_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        std_msgs
)

install(TARGETS
        right_angle_planner
        DESTINATION lib/${PROJECT_NAME}
)