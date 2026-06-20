#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 路径设置
    stack_share = get_package_share_directory('right_angle_stack_mix')
    track_share = get_package_share_directory('right_angle_track')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_name = 'right_angle_world'
    world_path = os.path.join(track_share, 'worlds', 'right_angle_harmonic.sdf')
    robot_sdf = os.path.join(stack_share, 'models', 'right_angle_car_harmonic', 'model.sdf')
    robot_xacro = os.path.join(stack_share, 'urdf', 'right_angle_car.urdf.xacro')
    rviz_config = os.path.join(stack_share, 'rviz', 'right_angle.rviz')
    stack_config = os.path.join(stack_share, 'config', 'right_angle_stack.yaml')
    track_sdf = os.path.join(track_share, 'models', 'shixi', 'model.sdf')
    track_models = os.path.join(track_share, 'models')
    stack_models = os.path.join(stack_share, 'models')

    # 启动参数
    model_name = LaunchConfiguration('model_name')
    gz_args = LaunchConfiguration('gz_args')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_builtin_perception = LaunchConfiguration('use_builtin_perception')
    perception_map_topic = LaunchConfiguration('perception_map_topic')
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    gz_resource_path = [
        track_models,
        os.pathsep,
        stack_models,
        os.pathsep,
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='racecar'),

        # -r 立即启动
        # -s 启动服务
        # -v 4 最详细调试信息
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r -v 4 ', world_path],
            description='Arguments passed to Gazebo Sim.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_builtin_perception', default_value='true'),
        DeclareLaunchArgument('perception_map_topic', default_value='/perception/cones'),

        # 环境变量
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),

        # Gazebo 仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args, }.items(),
        ),

        # 生成车辆（初始位姿：x=0, y=-15, z=0, yaw=90°=朝北）
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_right_angle_car',
            output='screen',
            arguments=[
                '-world', world_name,
                '-file', robot_sdf,
                '-name', model_name,
                '-x', '0.0',
                '-y', '-15.0',
                '-z', '0.0',
                '-Y', '1.57079632679',
            ],
        ),

        # Gazebo ↔ ROS 桥接
        # gazebo的仿真时间与ROS时间桥接
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_clock_bridge',
            output='screen',
            arguments=[
                f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
        ),
        # Gazebo与ROS传感器、控制之间的桥接
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # ROS 发布控制
                '/sensors/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',  # gazebo 发布里程计
                '/sensors/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',  # gazebo 发布IMU数据
                '/sensors/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',  # gazebo 发布GPS数据
                '/sensors/magnetic_field@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',  # gazebo 发布磁力计数据
                '/sensors/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', # gazebo 发布激光数据
                '/sensors/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked', # gazebo 发布激光点云数据
                '/sensors/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', # gazebo 发布相机图像数据
                '/sensors/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', # gazebo 发布相机信息
            ],
        ),

        # TF 发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', robot_xacro]),
                'use_sim_time': sim_time_param,
            }],
        ),

        # 定位融合
        Node(
            package='right_angle_stack_mix',
            executable='localization_fusion',
            name='localization_fusion',
            output='screen',
            parameters=[stack_config, {'use_sim_time': sim_time_param}],
        ),
        # 感知
        Node(
            package='right_angle_stack_mix',
            executable='track_perception',
            name='track_perception',
            output='screen',
            parameters=[stack_config, {
                'track_sdf': track_sdf,
                'map_topic': perception_map_topic,
                'use_sim_time': sim_time_param,
            }],
            condition=IfCondition(use_builtin_perception),
        ),
        # 建图
        Node(
            package='right_angle_stack_mix',
            executable='cone_mapper',
            name='cone_mapper',
            output='screen',
            parameters=[stack_config, {
                'perception_map_topic': perception_map_topic,
                'use_sim_time': sim_time_param,
            }],
        ),
        # 规划
        Node(
            package='right_angle_stack_mix',
            executable='right_angle_planner',
            name='right_angle_planner',
            output='screen',
            parameters=[stack_config, {'use_sim_time': sim_time_param}],
        ),
        # 控制
        Node(
            package='right_angle_stack_mix',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[stack_config, {'use_sim_time': sim_time_param}],
        ),

        # RViz可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])