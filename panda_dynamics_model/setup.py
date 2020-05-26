#!/usr/bin/env python3

from setuptools import setup

try:
    from catkin_pkg.python_setup import generate_distutils_setup
    setup_args = generate_distutils_setup(
        packages=['pandadynamicsmodel'],
        package_dir={'': 'src'})
    setup(**setup_args)

except ImportError:
    setup(name='pandadynamicsmodel',
          version='0.1.0',
          description='Package to load the dynamics model of Franka Emika panda robot',
          author='Raphael Deimel, Jan Martin, Roman Kolbert',
          author_email='raphael.deimel@tu-berlin.de',
          url='http://www.mti-engage.tu-berlin.de/',
          packages=['pandadynamicsmodel'],
          package_dir={'': 'src'},
         )

