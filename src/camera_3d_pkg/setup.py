from setuptools import setup
from glob import glob
import os

package_name = 'camera_3d_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='3D Camera driver package for RGB-D cameras',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_3d_node = camera_3d_pkg.camera_3d_node:main',
            'pointcloud_node = camera_3d_pkg.pointcloud_node:main',
        ],
    },
)
