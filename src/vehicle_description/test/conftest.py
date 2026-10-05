
from pathlib import Path

import pytest
import yaml


@pytest.fixture(scope='module')
def lidar_config():
    package_dir = Path(__file__).resolve().parents[1]
    config_file = package_dir / 'config' / 'lidar.yaml'

    with config_file.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['lidar']


@pytest.fixture(scope='module')
def vehicle_config():
    package_dir = Path(__file__).resolve().parents[1]
    config_file = package_dir / 'config' / 'vehicle.yaml'

    with config_file.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['vehicle']