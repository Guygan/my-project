from setuptools import setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 motor driver using serial communication',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_control = motor_driver.motor_control_node:main',
        ],
    },
)

