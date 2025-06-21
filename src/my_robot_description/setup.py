from setuptools import find_packages, setup
import os

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index resource registration
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # URDF/Xacro files
        ('share/' + package_name + '/urdf', [
            os.path.join('urdf', 'four_wheel_bot.xacro')
        ]),

        # Launch files
        ('share/' + package_name + '/launch', [
            os.path.join('launch', 'display.launch.py'),
            os.path.join('launch', 'gazebo.launch.py')  
        ]),

        # World file
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_stop = my_robot_description.obstacle_stop:main',
        ],
    },
)

