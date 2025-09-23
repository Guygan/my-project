import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guygan',
    maintainer_email='guygan2002@gmail.com',
    description='Robot with RViz and Gazebo using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'laser_to_sonar = my_robot_description.laser_to_sonar_node:main',
        'frame_fixer = my_robot_description.frame_fixer:main',
    ],
},
)
