from setuptools import setup

package_name = 'trajectory_generator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sajeed',
    maintainer_email='none@example.com',
    description='Trajectory generator node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_generator_node = trajectory_generator.trajectory_generator_node:main',
        ],
    },
)
