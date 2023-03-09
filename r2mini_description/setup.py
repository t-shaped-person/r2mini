import os # add
from glob import glob # add
from setuptools import setup

package_name = 'r2mini_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # add
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*.dae', recursive=True)), # add
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')), # add
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')), # add

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr.K',
    maintainer_email='kucira00@gmail.com',
    description='3D models of simulation and visualization for r2mini.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
