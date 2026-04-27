import glob
import os
from setuptools import find_packages, setup

package_name = 'homo_multirobot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'),
            glob.glob('maps/.gitkeep')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='l1anggmgo',
    maintainer_email='1528994924@qq.com',
    description='Navigation and AMCL localization for multi-robot systems',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                        'lifecycle_activator = homo_multirobot_nav.lifecycle_activator:main',
            'tf_static_bridge = homo_multirobot_nav.tf_static_bridge:main',
            'scan_transformer = homo_multirobot_nav.scan_transformer:main',
            'tf_debug = homo_multirobot_nav.tf_debug:main',
            'python_amcl = homo_multirobot_nav.python_amcl:main',
        ],
    },
)
