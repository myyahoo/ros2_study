import os
from setuptools import find_packages, setup
import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tech',
    maintainer_email='tech@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'pub_node = my_package.pub_node:main',
            'sub_node = my_package.sub_node:main',
            'video_person_detection = my_package.video_person_detection_node:main',
            'cmd_pose = my_package.cmd_pose:main',
            'my_service = my_package.my_service:main',
            'my_action = my_package.my_action:main',
        ],
    },
)
