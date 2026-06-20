import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'right_angle_stack'


def package_files(subdir, pattern):
    return glob(os.path.join(subdir, pattern))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch', '*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), package_files('urdf', '*.xacro')),
        (os.path.join('share', package_name, 'rviz'), package_files('rviz', '*.rviz')),
        (os.path.join('share', package_name, 'config'), package_files('config', '*.yaml')),
        # 车辆 SDF 也要安装，否则 ros_gz_sim create 找不到。
        (
            os.path.join('share', package_name, 'models', 'right_angle_car_harmonic'),
            package_files(os.path.join('models', 'right_angle_car_harmonic'), 'model.*'),
        ),
        (
            os.path.join('share', package_name, 'models', 'right_angle_car_wsl_headless'),
            package_files(os.path.join('models', 'right_angle_car_wsl_headless'), 'model.*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCUT Racing',
    maintainer_email='user@example.com',
    description='Right-angle Gazebo simulation stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_mapper = right_angle_stack.cone_mapper:main',
            'localization_fusion = right_angle_stack.localization_fusion:main',
            'pure_pursuit_controller = right_angle_stack.pure_pursuit_controller:main',
            'right_angle_planner = right_angle_stack.right_angle_planner:main',
            'sim_sensor_bridge = right_angle_stack.sim_sensor_bridge:main',
            'track_perception = right_angle_stack.track_perception:main',
        ],
    },
)
