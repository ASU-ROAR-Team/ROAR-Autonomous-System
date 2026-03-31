"""
Install roscan library
"""
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["roscan"], package_dir={"": "src"})

setup(**d)
