"""The common model must work without looking up any simulator package."""
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


def test_hardware_model_does_not_resolve_simulator_packages():
    model = Path(get_package_share_directory('vehicle_description')) / 'urdf/vehicle.urdf.xacro'
    def no_package_lookup(*args, **kwargs):
        raise AssertionError(f'Hardware xacro attempted a package lookup: {args}')
    with patch('ament_index_python.packages.get_package_share_directory', no_package_lookup):
        robot = ET.fromstring(xacro.process_file(str(model)).toxml())
    assert not robot.findall('./gazebo/plugin')
    assert not robot.findall('./gazebo/sensor')
    assert robot.find("./link[@name='laser_frame']") is not None
    assert robot.find("./joint[@name='base_footprint_joint']/parent").get('link') == 'base_footprint'
    for axle in ('front', 'rear'):
        for side, sign in (('left', 1), ('right', -1)):
            origin = robot.find(f"./joint[@name='{axle}_{side}_wheel_joint']/origin")
            assert float(origin.get('xyz').split()[1]) == pytest.approx(sign * .1203 / 2)
