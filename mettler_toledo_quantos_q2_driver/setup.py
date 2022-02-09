#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mettler_toledo_quantos_q2_driver'],
    package_dir={'': 'src'}
)

setup(**d)
