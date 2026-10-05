from setuptools import find_packages, setup
from glob import glob

package_name = 'vehicle_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='vehicle_team',
    maintainer_email='maintainer@example.com',
    description='Vehicle localization and state estimation.',
    license='Apache-2.0',
)
