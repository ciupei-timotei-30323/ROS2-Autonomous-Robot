from setuptools import find_packages, setup

package_name = 'delivery_robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/Locations.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='timoteiciupei@gmail.com',
    description='Core functionality for the Delivery Robot, including mission coordination and TCP bridge',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_coordinator = delivery_robot_core.mission_coordinator_node:main',
            'tcp_bridge_node = delivery_robot_core.tcp_bridge_node:main',
            'waypoint_logger = delivery_robot_core.waypoint_logger_node:main',
            # 'odom_to_euler = delivery_robot_core.odom_to_euler_node:main'
        ],
    },
)
