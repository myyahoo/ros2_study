from setuptools import setup

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='user',
    author_email='user@todo.todo',
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='로봇 암 제어 및 Vision 기반 작업 수행',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = robot_arm_control.vision_node:main',
            'arm_control_node = robot_arm_control.arm_control_node:main',
            'task_executor_node = robot_arm_control.task_executor_node:main',
        ],
    },
)
