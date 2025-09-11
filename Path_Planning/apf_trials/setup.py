from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apf_trials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('obstacleParams/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar@todo.todo',
    description='The apf_trials package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'APF_update = apf_trials.APF_update:main',
            'Obstacle_publisher = apf_trials.Obstacle_publisher:main',
        ],
    },
)
