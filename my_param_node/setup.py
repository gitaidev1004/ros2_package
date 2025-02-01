from setuptools import setup

package_name = 'my_param_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    author='ROS 2',
    author_email='ros2@example.com',
    description='ROS 2 Parameter Handling Example',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_node = my_param_node.param_node:main',
        ],
    },
)
