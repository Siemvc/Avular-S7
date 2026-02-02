from setuptools import find_packages, setup
import os
from glob import glob

# Zorg dat deze exact matcht met package.xml en je mapnaam
package_name = 'loader_control'

setup(
    name=package_name,
    version='0.0.0',
    # Dit zoekt automatisch naar de submap 'LoaderControl' met __init__.py
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # HIER ZIT DE TRUC VOOR JE LAUNCH FILES:
        # We kopiëren alles uit de map 'Launch' naar de installatie map 'share/LoaderControl/launch'
        (os.path.join('share', package_name, 'launch'), glob('Launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Avular',
    maintainer_email='user@todo.todo',
    description='Loader Control Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # loader_control (map) . bestandsnaam : functie
            'loader_node = loader_control.loader_control_node:main',
            'battery_node = loader_control.battery_voltage_node:main',
        ],
    },
)