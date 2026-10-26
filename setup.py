import os
from glob import glob
from setuptools import setup

package_name = 'ada526_robot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameer',
    maintainer_email='sameermubeen4@gmail.com',
    description='ADA526 5-DOF Robot Arm Vision, Kinematics, and Dynamixel Control Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_processor = ada526_robot_arm.img_processor:main',
            'robot_controller = ada526_robot_arm.robot_controller:main',
        ],
    },
)