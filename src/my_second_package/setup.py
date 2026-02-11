from setuptools import find_packages, setup

package_name = 'my_second_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_second.launch.py']),
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
            'pub_node = my_second_package.pub_node:main',
        ],
    },
)
