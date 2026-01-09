from setuptools import setup, find_packages
import os
from glob import glob

package_id = 'um982_driver'

setup(
    name=package_id,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_id]),
        ('share/' + package_id, ['package.xml']),
        (os.path.join('share', package_id, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_id, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 driver for Unicorecomm UM982 dual-antenna RTK GNSS receiver with integrated NTRIP client',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_node = um982_driver.um982_node:main',
        ],
    },
)
