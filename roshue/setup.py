#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roshue'],
    package_dir={'': 'src'},
    scripts=['scripts/roshue_bridge.py'],
)
setup(**d)
