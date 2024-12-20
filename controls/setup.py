import os 
from glob import glob # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
from setuptools import find_packages, setup # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
package_name = 'controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can = controls.can:main',
            'motor = controls.motor:main',

            'drivetrain = controls.drivetrain:main',
            'manipulator = controls.manipulator:main',
            'ik_manipulator = controls.ik_manipulator:main',
            'zero_ik_manipulator = controls.zero_ik_manipulator:main',

            
            'drivetrain_debugger = controls.drivetrain_debugger:main',
            'manipulator_debugger = controls.manipulator_debugger:main',
        ],
    },
)
