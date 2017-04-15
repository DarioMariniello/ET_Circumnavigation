from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ET_circumnavigation'],
    scripts=['nodes'],
    services=['srv'],
    package_dir={'': 'src'}
)

setup(**d)
