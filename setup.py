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
        (f'share/{pkg}/config', glob(os.path.join('config', '*.json5')))
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
            'new_robot_acknowledgement.py = gazebo_multisim_server.new_robot_acknowledgement:main',
            'topic_analyser_tui.py = gazebo_multisim_server.topic_analyser_tui:main',
            'horologist.py = gazebo_multisim_server.horologist:main',
            'repub_points.py = gazebo_multisim_server.repub_points:main',
            'topic_to_datum.py = gazebo_multisim_server.topic_to_datum:main',
            'datum_to_topic.py = gazebo_multisim_server.datum_to_topic:main',
            'spawn_entity_from_xml.py = gazebo_multisim_server.spawn_entity_from_xml:main'
        ],
    },
)
