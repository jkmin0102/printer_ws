from glob import glob
from os.path import isfile
from pathlib import Path
from setuptools import find_packages, setup

setup(
    name='namo_manipulation', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/namo_manipulation']),
                ('share/namo_manipulation', ['package.xml'])]
    + [('share/namo_manipulation/' + directory, [p for p in glob(directory + '/*') if isfile(p)])
       for directory in ('launch', 'models', 'config', 'tools', 'worlds', 'rviz', 'maps', 'docs', 'scripts')]
    + [('share/namo_manipulation/' + str(directory), [str(p) for p in directory.iterdir() if p.is_file()])
       for directory in [Path('models/official'), *Path('models/official').rglob('*')] if directory.is_dir()],
    package_data={'namo_manipulation': ['robot_geometry.json']},
    install_requires=['setuptools'], zip_safe=True, maintainer='EATED',
    maintainer_email='user@example.com', description='Physical NAMO manipulation simulation',
    license='Apache-2.0', entry_points={'console_scripts': [
        'physical_remover = namo_manipulation.arm_executor:main',
        'can_perception = namo_manipulation.can_perception:main',
        'mission = namo_manipulation.mission:main',
        'status_display = namo_manipulation.status_display:main',
        'drive_gate = namo_manipulation.drive_gate:main',
    ]},
)
