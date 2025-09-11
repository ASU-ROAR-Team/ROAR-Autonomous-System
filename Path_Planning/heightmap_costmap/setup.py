from setuptools import setup
import os
from glob import glob

package_name = 'heightmap_costmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name), ['path.csv']),
        ('share/' + package_name, glob('total_cost.csv')),
    ],
    install_requires=['setuptools', 'tqdm'],
    zip_safe=True,
    maintainer='abdelrhman',
    maintainer_email='abdelrhman@todo.todo',
    description='The heightmap_costmap package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_calibration = heightmap_costmap.map_calibration:main',
            'heightmap_to_costmap = heightmap_costmap.heightmap_to_costmap:main',
            'a_star = heightmap_costmap.a_star:main',
        ],
    },
)