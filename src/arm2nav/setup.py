from glob import glob
import os
from setuptools import find_packages, setup


package_name = 'arm2nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tzb',
    maintainer_email='tzb@example.com',
    description='10 Hz arm-to-navigation communication and sentry perception node.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'arm2nav = arm2nav.node:main',
        ],
    },
)
