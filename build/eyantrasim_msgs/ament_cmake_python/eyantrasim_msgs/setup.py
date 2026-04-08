from setuptools import find_packages
from setuptools import setup

setup(
    name='eyantrasim_msgs',
    version='1.10.1',
    packages=find_packages(
        include=('eyantrasim_msgs', 'eyantrasim_msgs.*')),
)
