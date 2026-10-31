from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/models/roam_rover',
         ['../../models/roam_rover/model.sdf']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Arham',
    maintainer_email='arham@example.com',
    description='Simulated rover sensors, frames, and wheel encoder nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_encoder_publisher = '
            'my_robot.wheel_encoder_publisher:main',
            'wheel_encoder_subscriber = '
            'my_robot.wheel_encoder_subscriber:main',
            'rover_sensor_state = my_robot.rover_state:main',
        ],
    },
)
