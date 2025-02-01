from setuptools import setup

package_name = 'tf2_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    author='ROS 2',
    author_email='ros2@example.com',
    description='TF2 Broadcasting and Listening Example',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_broadcaster_listener = tf2_example.tf2_broadcaster_listener:main',
        ],
    },
)
