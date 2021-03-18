#!/usr/bin/env python3
# 2021/03/18 hato #!/usr/bin/env python -> !/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rovi_utils'],
    package_dir={'': 'src'},
    scripts=['script/tf_lookup.py','script/tf_euler.py']
)

setup(**d)
