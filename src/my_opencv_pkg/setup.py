import glob
from setuptools import find_packages, setup
import os

package_name = 'my_opencv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tech',
    maintainer_email='tech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = my_opencv_pkg.my_node:main',
            'img_pub = my_opencv_pkg.img_pub:main',
            'img_sub = my_opencv_pkg.img_sub:main',
        ],
    },
)
