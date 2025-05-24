# pylint: disable=all
# mypy: ignore-errors
"""
Install simple pure pursuit library
"""
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["supervisor"], package_dir={"": "src"})

setup(**d)
