import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'custom_ur_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dat-ubuntu2204',
    maintainer_email='trandoanquang483@gmail.com',
    description='CSV logger chung + node gom dữ liệu camera + AAPF.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_aapf_logger = custom_ur_logger.human_aapf_logger:main',
        ],
    },
)
