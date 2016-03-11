from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[],
    scripts=['scripts/barrett_hand_interface.py', 'scripts/markers.py'],
    package_dir={'': ''}
)

setup(**d)
