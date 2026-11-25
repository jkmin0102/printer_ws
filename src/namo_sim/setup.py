from glob import glob
from os.path import isfile
from setuptools import find_packages, setup

package_name = 'namo_sim'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + [('share/' + package_name + '/' + directory, [p for p in glob(directory + '/*') if isfile(p)])
         for directory in ('launch', 'config', 'maps', 'worlds', 'models', 'rviz', 'tools')],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EATED',
    maintainer_email='user@example.com',
    description='Isolated EATED NAMO simulation.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'velocity_gate = namo_sim.velocity_gate:main',
        'scan_clearing = namo_sim.scan_clearing:main',
        'baseline = namo_sim.baseline:main',
        'mission = namo_sim.mission:main',
        'mock_remover = namo_sim.mock_remover:main',
    ]},
)
