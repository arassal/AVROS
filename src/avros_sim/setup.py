import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'avros_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.wbt'))
            + glob(os.path.join('worlds', '.*.wbproj'))),
        (os.path.join('share', package_name, 'resource'),
            glob(os.path.join('resource', '*.urdf'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AV Lab',
    maintainer_email='avlab@cpp.edu',
    description='Webots simulation environment for AVROS',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
