import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'operations'

setup(
    name=package_name,
    version='2.1.5',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
                                                        'tunnel_exploration.launch.py'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Boringtao'],
    author_email=['xinghui.tao@robominds.ai'],
    keywords=['ROS2', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Turtlebot3 Tunnel Exploration.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'tunnel_exploration = \
                operations.tunnel_exploration.main:main',
        ],
    },
)
