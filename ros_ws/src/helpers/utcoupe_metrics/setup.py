#! /usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['utcoupe_metrics', 'utcoupe_metrics.metric_types'],
    package_dir={'': 'src_py'})

setup(**setup_args)
