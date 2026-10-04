from glob import glob

from setuptools import find_packages, setup

package_name = 'vehicle_description'
setup(
    name=package_name, version='0.0.1', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Install only the derived binary meshes; keep the large source STL in the workspace.
        ('share/' + package_name + '/meshes',
        glob('meshes/chassis_body.stl') + glob('meshes/*_wheel.stl')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='vehicle_team',
    maintainer_email='maintainer@example.com',
    description='URDF, meshes, and RViz resources for the 4WD vehicle.',
    license='Apache-2.0',
)
