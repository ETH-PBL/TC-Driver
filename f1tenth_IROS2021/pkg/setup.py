## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pkg'],
    package_dir={'': 'src'}
)
setup(**d)