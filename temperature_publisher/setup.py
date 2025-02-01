from setuptools import setup

package_name = 'temperature_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    author='ROS 2',
    author_email='ros2@example.com',
    description='Publish random temperature data',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp_publisher = temperature_publisher.temp_publisher:main',
        ],
    },
)
