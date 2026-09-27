from setuptools import find_packages, setup

package_name = 'vehicle_drivers'

setup(name=package_name, version='0.0.1', packages=find_packages(), data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
], install_requires=['setuptools'], zip_safe=True,
      maintainer='vehicle_team', maintainer_email='maintainer@example.com',
      description='Hardware drivers for vehicle actuators and sensors.', license='Apache-2.0')
