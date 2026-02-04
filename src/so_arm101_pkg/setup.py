from setuptools import setup, find_packages

package_name = 'so_arm101_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_control.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
        'numpy>=1.21.0',
    ],
    zip_safe=True,
    author='ROS Developer',
    author_email='ros@example.com',
    maintainer='ROS Developer',
    maintainer_email='ros@example.com',
    keywords=['ROS2', 'Robot Arm', '6DOF'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development',
    ],
    description='6DOF Robot Arm Control Package',
    long_description='A ROS2 package for controlling a 6DOF robot arm via serial communication.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'arm_controller = so_arm101_pkg.arm_controller:main',
            'arm_keyboard = so_arm101_pkg.arm_keyboard:main',
            'arm_joy = so_arm101_pkg.arm_joystick:main',
            'arm_demo = so_arm101_pkg.arm_demo:main',
        ],
    },
)
