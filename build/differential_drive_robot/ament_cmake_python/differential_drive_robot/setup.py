from setuptools import find_packages
from setuptools import setup

setup(
    name='differential_drive_robot',
    version='0.1.0',
    packages=find_packages(
        include=('differential_drive_robot', 'differential_drive_robot.*')),
)
