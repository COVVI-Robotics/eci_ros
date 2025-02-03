import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'covvi_hand_driver'

setup(
    name='covvi_hand_driver',
    version='1.1.6',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', 'covvi_hand_driver')]),
        (os.path.join('share', 'covvi_hand_driver'), ['package.xml']),
        (os.path.join('share', 'covvi_hand_driver', 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=[
        'setuptools',
        'covvi-eci==1.1.6',
    ],
    zip_safe=True,
    maintainer='Jordan Birdsall',
    maintainer_email='jordan.birdsall@covvi.com',
    description='A package to provide the Covvi Hand Driver Nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = covvi_hand_driver.covvi_server_node:main',
            'client = covvi_hand_driver.covvi_client_node:main',
        ],
    },
)