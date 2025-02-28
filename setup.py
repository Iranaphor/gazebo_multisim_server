from setuptools import setup
from glob import glob
import os

package_name = 'gazebo_multisim_server'
pkg = package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'topics_to_spawnservice.py = gazebo_multisim_server.topics_to_spawnservice:main',
            'spawnservice_to_topic.py = gazebo_multisim_server.spawnservice_to_topic:main',
            'validate_model.py = gazebo_multisim_server.validate_model:main',
        ],
    },
)
