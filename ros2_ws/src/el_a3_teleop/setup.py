from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'el_a3_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wy',
    maintainer_email='wy@todo.todo',
    description='EL-A3 robot arm Xbox controller real-time control package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'xbox_teleop_node = el_a3_teleop.xbox_teleop_node:main',
            'xbox_servo_node = el_a3_teleop.xbox_servo_node:main',
            'joycon_imu_teleop = el_a3_teleop.joycon_imu_teleop_node:main',
            'joycon_imu_driver = el_a3_teleop.joycon_imu_driver:main',
            'joycon_visualizer = el_a3_teleop.joycon_visualizer_node:main',
            'master_slave_node = el_a3_teleop.master_slave_node:main',
        ],
    },
)
