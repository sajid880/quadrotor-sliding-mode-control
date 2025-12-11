from setuptools import setup
from glob import glob
import os

package_name = 'drone_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sajeed',
    maintainer_email='none@example.com',
    description='Offboard helper and launch',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_node = drone_interface.offboard_node:main',
        ],
    },
)
