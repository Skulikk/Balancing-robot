from setuptools import find_packages
from setuptools import setup

setup(
    name='sensors_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('sensors_pkg', 'sensors_pkg.*')),
)
