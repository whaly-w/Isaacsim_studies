from setuptools import find_packages
from setuptools import setup

setup(
    name='h1_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('h1_msgs', 'h1_msgs.*')),
)
