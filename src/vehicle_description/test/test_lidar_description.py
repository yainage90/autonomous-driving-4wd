import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


@pytest.fixture(scope='module')
def robot():
    package_dir = Path(__file__).resolve().parents[1]
    xacro_file = package_dir / 'urdf' / 'vehicle.urdf.xacro'

    document = xacro.process_file(str(xacro_file))
    return ET.fromstring(document.toxml())


@pytest.fixture(scope='module')
def lidar(robot):
    sensors = robot.findall(
        "./gazebo[@reference='laser_frame']/sensor"
    )

    assert len(sensors) == 1, (
        'laser_frame에는 라이다 센서가 하나 있어야 합니다.'
    )
    return sensors[0]


def test_lidar_attachment(robot, lidar):
    assert robot.find("./link[@name='laser_frame']") is not None
    assert lidar.get('type') == 'gpu_lidar'
    assert lidar.findtext('topic') == '/scan'
    assert lidar.findtext('gz_frame_id') == 'laser_frame'

    pose = [float(value) for value in lidar.findtext('pose').split()]
    assert pose == pytest.approx([0.0] * 6)


def test_lidar_config_is_valid(lidar_config):
    numeric_keys = (
        'update_rate_hz',
        'min_angle_rad',
        'max_angle_rad',
        'min_range_m',
        'max_range_m',
        'range_resolution_m',
    )

    for key in numeric_keys:
        value = lidar_config[key]
        assert type(value) in (int, float), f'{key}는 숫자여야 합니다.'
        assert math.isfinite(value), f'{key}는 유한한 값이어야 합니다.'

    samples = lidar_config['horizontal_samples']
    assert type(samples) is int
    assert samples >= 2

    assert lidar_config['update_rate_hz'] > 0
    assert lidar_config['range_resolution_m'] > 0
    assert (
        0 < lidar_config['min_range_m']
        < lidar_config['max_range_m']
    )
    assert (
        lidar_config['min_angle_rad']
        < lidar_config['max_angle_rad']
    )


def test_lidar_is_2d(lidar):
    # 2D 라이다라는 구조는 설정 파일과 독립적으로 검사
    vertical = lidar.find('lidar/scan/vertical')
    assert vertical is not None
    assert int(vertical.findtext('samples')) == 1
    assert float(vertical.findtext('resolution')) == pytest.approx(1.0)
    assert float(vertical.findtext('min_angle')) == pytest.approx(0.0)
    assert float(vertical.findtext('max_angle')) == pytest.approx(0.0)
