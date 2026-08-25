from setuptools import find_packages, setup

package_name = 'my_robot_gripper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gripper.yaml']),
        ('share/' + package_name + '/launch', ['launch/gripper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Repository maintainer',
    maintainer_email='root@todo.todo',
    description='Smooth servo control for the robot gripper and lift.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_controller = my_robot_gripper.gripper_controller:main',
        ],
    },
)
