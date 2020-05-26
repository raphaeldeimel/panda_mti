#!/usr/bin/env python3


from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mti_pdcontrollergoal_publisher'],
    package_dir={'': 'scripts'}
)

setup(**d)
