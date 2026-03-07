from setuptools import setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Avinash',
    maintainer_email='avinashlakhwani0104@gmail.com',
    description='F1Tenth Pure Pursuit Lab',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = control.control:main',
            'logger = control.waypoint:main',
            'teleop = control.ackermann_teleop:main',
            'marker = control.marker:main',
        ],
    },
)