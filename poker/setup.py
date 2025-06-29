from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'poker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
        (os.path.join('share', package_name, 'assets'), glob('assets/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyle Vickstrom',
    maintainer_email='vickskyl@oregonstate.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poker_console = poker.poker_console:createConsole',
            'visualizeTable = poker.visualizeTableGUI:main',
            'poker_game = poker.poker_game:createGame',
            'poker_camera = poker.poker_camera:main',
            'camera_viewer = poker.camera_viewer:main',
            'img_manip = poker.image_manip:main',
            'poker_arduino = poker.poker_arduino:main',
            'save_game = poker.save_game:main'
        ],
    },
)
