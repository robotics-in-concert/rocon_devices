#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_hue'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_hue_bridge.py'],
)
setup(**d)
