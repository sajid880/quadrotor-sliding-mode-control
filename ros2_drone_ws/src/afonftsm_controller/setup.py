from setuptools import setup
from glob import glob
import os

package_name = 'afonftsm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files (if you add any later)
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sajeed',
    maintainer_email='none@example.com',
    description='AFONFTSM controller node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'afonftsm_controller_node = afonftsm_controller.afonftsm_controller_node:main',
        ],
    },
)
