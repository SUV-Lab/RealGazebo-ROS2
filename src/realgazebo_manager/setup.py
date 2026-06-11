import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'realgazebo_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmk',
    maintainer_email='kmk6061602@naver.com',
    description='RealGazebo runtime spawn/lifecycle manager',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manager_node = realgazebo_manager.manager_node:main',
        ],
    },
)
