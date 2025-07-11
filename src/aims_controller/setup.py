import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aims_controller'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ian Hunt',
    author_email='hutzuonline@yahoo.com',
    maintainer='Ian Hunt',
    maintainer_email='hutzuonline@yahoo.com',
    description='A Python GUI plugin for displaying ROS AIMS integration.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'controller_node = aims_controller.controller_node:main',
            'joystick_node = aims_controller.joystick_node:main',
            'cv_node = aims_controller.cv_node:main',
        ],
    },
)