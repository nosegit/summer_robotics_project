from setuptools import find_packages, setup


package_name = 'my_robot_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/mission.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Repository maintainer',
    maintainer_email='root@todo.todo',
    description='Sequence navigation and parcel acquisition missions.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'led_controller = my_robot_mission.utils.led_controller:main',
            'parcel_mission = my_robot_mission.parcel_mission:main',
        ],
    },
)
