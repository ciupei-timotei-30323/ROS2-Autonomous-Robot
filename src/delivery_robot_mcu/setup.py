from setuptools import find_packages, setup

package_name = 'delivery_robot_mcu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='timoteiciupei@gmail.com',
    description='Package for communication with the MCU for the Delivery Robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mcu_interface = delivery_robot_mcu.mcu_interface_node:main'
        ],
    },
)
