add_executable(pure_pursuit_controller
        pure_pursuit_controller.cpp
        main.cpp
)
ament_target_dependencies(pure_pursuit_controller
        rclcpp
        geometry_msgs
        nav_msgs
        std_msgs
        visualization_msgs
)

install(TARGETS
        pure_pursuit_controller
        DESTINATION lib/${PROJECT_NAME}
)