import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'custom_ur_human_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dat-ubuntu2204',
    maintainer_email='dat.nt216082@sis.hust.edu.vn',
    description='Human arm tracking (MediaPipe + AprilTag) node for the UR3 setup.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_arm_node = custom_ur_human_tracking.real_arm_node:main',
            'human_obstacle_bridge = custom_ur_human_tracking.human_obstacle_bridge:main',
        ],
    },
)
