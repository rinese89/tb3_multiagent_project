import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tb30_multi_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'main_master_multi.launch.py'))),
    (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'turtlebot3_state_publisher.launch.py'))),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'ld08.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'turtlebot3_ros=turtlebot3_node.turtlebot3_ros:main',
                'control_trajectory=tb30_multi_pkg.control_trajectory:main',
                'broadcaster_reference_0=tb30_multi_pkg.broadcaster_reference_0:main',
                'central_control_node_0=tb30_multi_pkg.central_control_node_0:main'

        ],
    },
)


