
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope='module')
def lidar_config():
    package_dir = Path(__file__).resolve().parents[1]
    config_file = package_dir / 'config' / 'lidar.yaml'

    with config_file.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['lidar']


@pytest.fixture(scope='module')
def vehicle_config():
    package_dir = Path(__file__).resolve().parents[1]
    config_file = Path(get_package_share_directory('vehicle_description')) / 'config' / 'vehicle.yaml'

    with config_file.open(encoding='utf-8') as stream:
        vehicle = yaml.safe_load(stream)['vehicle']
    with (package_dir / 'config' / 'simulation.yaml').open() as stream:
        vehicle['calibration'] = yaml.safe_load(stream)['simulation']
    return vehicle