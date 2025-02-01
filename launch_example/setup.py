from setuptools import setup

package_name = 'launch_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'example_interfaces'],
    zip_safe=True,
    author='ROS 2',
    author_email='ros2@example.com',
    description='ROS 2 Launch Example',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = launch_example.talker:main',
            'listener = launch_example.listener:main',
        ],
    },
)
