from setuptools import setup
import os
from glob import glob

package_name = 'ddsm115_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_motor_id = ddsm115_controller.check_motor_id:main',
            'set_motor_id = ddsm115_controller.set_motor_id:main',
            'velocity_control = ddsm115_controller.velocity_control:main',
            'four_wheels_robot = ddsm115_controller.four_wheels_robot:main',
        ],
    },
)
