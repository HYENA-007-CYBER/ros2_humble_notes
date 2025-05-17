from setuptools import find_packages, setup

package_name = 'week1_pkg'

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
    maintainer='hyensteen',
    maintainer_email='hyensteen10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'first_node = week1_pkg.first_node:main',
            'number_publisher= week1_pkg.number_publisher:main',
            'square_subscriber= week1_pkg.square_subscriber:main',

        ],
    },
)
