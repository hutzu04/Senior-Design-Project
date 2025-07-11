import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rqt_aims'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml and plugin.xml files
        (os.path.join('share', package_name), ['package.xml', 'plugin.xml']),
        # Include the AIMS_2.ui file in the resource folder
        (os.path.join('share', package_name, 'resource'), ['resource/AIMS_2.ui']),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ian Hunt',
    author_email='hutzuonline@yahoo.com',
    maintainer='Ian Hunt',
    maintainer_email='hutzuonline@yahoo.com',
    keywords=['foo', 'bar'],
    description='A Python GUI plugin for displaying ROS AIMS integration.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'convert_g_to_can = rqt_aims.convert_g_to_can:main',
            'send_g_code = rqt_aims.send_g_code:main',
        ],
    },
)