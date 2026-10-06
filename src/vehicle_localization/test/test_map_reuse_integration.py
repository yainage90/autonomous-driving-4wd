
import math
import os
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@pytest.mark.skipif(
    os.environ.get('RUN_MAP_REUSE_INTEGRATION') != '1',
    reason='활성화된 Map Server가 필요합니다.',
)
def test_saved_map_is_published():
    share = Path(get_package_share_directory('vehicle_localization'))
    with (share / 'maps' / 'evaluation.yaml').open(
        encoding='utf-8'
    ) as stream:
        metadata = yaml.safe_load(stream)

    rclpy.init()
    node = rclpy.create_node('map_reuse_integration_test')
    received = []

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscription = node.create_subscription(
        OccupancyGrid, '/map', received.append, qos
    )

    try:
        deadline = time.monotonic() + 10.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received, '10초 안에 저장된 지도를 받지 못했습니다.'
        grid = received[0]

        assert grid.header.frame_id == 'map'
        assert grid.info.resolution == pytest.approx(
            metadata['resolution']
        )
        assert grid.info.origin.position.x == pytest.approx(
            metadata['origin'][0]
        )
        assert grid.info.origin.position.y == pytest.approx(
            metadata['origin'][1]
        )

        yaw = metadata['origin'][2]
        orientation = grid.info.origin.orientation
        assert orientation.x == pytest.approx(0.0)
        assert orientation.y == pytest.approx(0.0)
        assert orientation.z == pytest.approx(math.sin(yaw / 2))
        assert orientation.w == pytest.approx(math.cos(yaw / 2))

        # 지도를 다시 작성해도 유효한 지도 수신 여부를 확인한다.
        assert grid.info.width > 0
        assert grid.info.height > 0
        assert len(grid.data) == grid.info.width * grid.info.height
        assert 0 in grid.data, '빈 공간이 없습니다.'
        assert 100 in grid.data, '점유 영역이 없습니다.'

    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
