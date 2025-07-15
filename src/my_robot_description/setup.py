from setuptools import find_packages, setup
import os

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            os.path.join('urdf', 'four_wheel_bot.xacro')
        ]),
        ('share/' + package_name + '/launch', [
            os.path.join('launch', 'display.launch.py'),
            os.path.join('launch', 'gazebo.launch.py'),
            os.path.join('launch', 'gazebo_yolo.launch.py')
        ]),
        ('share/' + package_name + '/worlds', [
            os.path.join('worlds', 'my_world.world')
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyensteen',
    maintainer_email='hyensteen10@gmail.com',
    description='4-wheeled differential-drive robot description package',
    license='MIT',
    
    entry_points={
        'console_scripts': [
            'obstacle_stop = my_robot_description.obstacle_stop:main',
            'cone_detector = my_robot_description.opencv_cone_node:main',
            'yolo_cone_detector = my_robot_description.cone_detect_yolo:main',
        ],
    },
)
