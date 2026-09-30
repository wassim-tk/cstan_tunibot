import os
from glob import glob

from setuptools import setup

package_name = 'cstam_core'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CSTAM TuniBot Software Team',
    maintainer_email='dev@cstam.tunibot.org',
    description='Delivery task management and auto-docking for the CSTAM service robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'delivery_task_manager = cstam_core.delivery_task_manager:main',
            'docking_controller = cstam_core.docking_controller:main',
            'request_delivery = cstam_core.request_delivery:main',
            'mapping_tour = cstam_core.mapping_tour:main',
            'save_map = cstam_core.map_saver:main',
        ],
    },
)
