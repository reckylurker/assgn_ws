from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_dwa'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanish Jain',
    maintainer_email='recklurker@gmail.com',
    description='10xConstructionAssignment',
    license='Please Hire Me',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 
            'launch_dwa = custom_dwa.dwa:main',
        ],
    },
)
