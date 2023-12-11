import os
from glob import glob
from setuptools import setup

package_name = 'state_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
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
        				'joint_command_publisher = state_visualizer.state_publisher:main',
        				'prize_marker_publisher = state_visualizer.visualize_prizes:main',
        ],
    },
)
