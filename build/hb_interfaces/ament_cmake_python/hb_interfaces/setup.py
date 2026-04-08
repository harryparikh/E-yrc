from setuptools import find_packages
from setuptools import setup

setup(
    name='hb_interfaces',
    version='1.10.1',
    packages=find_packages(
        include=('hb_interfaces', 'hb_interfaces.*')),
)
