cmake_minimum_required(VERSION 3.8)
project(right_angle_track)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        models
        meshes
        worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
