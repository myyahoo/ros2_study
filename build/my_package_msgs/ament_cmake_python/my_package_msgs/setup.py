from setuptools import find_packages
from setuptools import setup

setup(
    name='my_package_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('my_package_msgs', 'my_package_msgs.*')),
)
