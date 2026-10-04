from setuptools import find_packages, setup
from glob import glob


package_name = 'vehicle_simulation'
setup(
    name=package_name, version='0.0.1', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vehicle_team',
    maintainer_email='maintainer@example.com',
    description='Simulation and mock interfaces for the 4WD vehicle.',
    license='Apache-2.0'
)
