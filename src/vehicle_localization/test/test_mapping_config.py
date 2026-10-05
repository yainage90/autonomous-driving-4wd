from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def test_installed_mapping_files():
    share = Path(
        get_package_share_directory('vehicle_localization')
    )

    assert (share / 'launch' / 'mapping.launch.py').is_file()

    config_path = share / 'config' / 'slam.yaml'
    assert config_path.is_file()

    with config_path.open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['slam_toolbox']['ros__parameters']

    # 기존 차량의 인터페이스와 일치해야 합니다.
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_footprint'
    assert params['map_frame'] == 'map'
    assert params['scan_topic'] == '/scan'
    assert params['mode'] == 'mapping'

    # 지도와 TF를 생성할 수 있는 설정인지 확인합니다.
    assert params['resolution'] > 0.0
    assert params['transform_publish_period'] > 0.0
    assert params['map_update_interval'] > 0.0
    assert 0.0 <= params['min_laser_range'] < params['max_laser_range']
