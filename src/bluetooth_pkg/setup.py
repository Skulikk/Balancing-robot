from setuptools import setup
import os
from glob import glob

package_name = 'bluetooth_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skulikk',
    maintainer_email='skulikk@todo.todo',
    description='Bluetooth control node for robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluetooth_node = bluetooth_pkg.bluetooth_node:main',
        ],
    },
)