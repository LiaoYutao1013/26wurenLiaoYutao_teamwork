from setuptools import find_packages, setup

package_name = 'sim_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'sim_perception': ['pyarmor_runtime_000000/*'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='yijiewang',
    maintainer_email='1103477790@qq.com',
    description='Simulated cone perception node for the right-angle track task.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_node = sim_perception.sim_node:main',
        ],
    },
)
