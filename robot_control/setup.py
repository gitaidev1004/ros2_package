from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'turtlebot3_msgs'],
    zip_safe=True,
    author='ROS 2',
    author_email='ros2@example.com',
    description='Control TurtleBot3 using ROS 2',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = robot_control.robot_controller:main',
        ],
    },
)
