from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'manipulator_driver_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=['manipulator_driver_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epon',
    maintainer_email='epon@todo.todo',
    description='Simulated manipulator with hand-written controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_controller=manipulator_driver_sim.my_controller:main",
            "move2pose=manipulator_driver_sim.move2pose_server:main"
        ],
    },
)
