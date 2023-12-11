import os
from glob import glob
from setuptools import setup

package_name = 'motion_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'json'), glob(os.path.join('json', '*json*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anna-lee',
    maintainer_email='mcleanannalee@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        				'planner = motion_planner.plan_motion_steps:main',
        ],
    },
)
