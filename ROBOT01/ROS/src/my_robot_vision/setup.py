from setuptools import find_packages, setup


package_name = 'my_robot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/launch', [
            'launch/pi_camera.launch.py',
            'launch/rtp_camera.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Repository maintainer',
    maintainer_email='root@todo.todo',
    description='Camera drivers and vision pipelines for the mecanum robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pi_camera_publisher = my_robot_vision.pi_camera_publisher:main',
            'rtp_camera_publisher = my_robot_vision.rtp_camera_publisher:main',
            'convert_camera_calibration = '
            'my_robot_vision.convert_npz_to_camera_info_yaml:main',
        ],
    },
)
