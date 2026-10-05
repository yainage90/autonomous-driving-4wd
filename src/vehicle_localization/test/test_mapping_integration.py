import os
import time

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


@pytest.mark.skipif(
    os.environ.get('RUN_MAPPING_INTEGRATION') != '1',
    reason='실행 중인 Gazebo와 SLAM Toolbox가 필요합니다.',
)
def test_map_and_scan_time_tf():
    rclpy.init()

    node = rclpy.create_node(
        'mapping_integration_test',
        parameter_overrides=[
            Parameter('use_sim_time', value=True),
        ],
    )

    maps = []
    scans = []

    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    # 첫 스캔을 보관해, TF가 그 시각까지 도착할 시간을 줍니다.
    def receive_scan(msg):
        if not scans:
            scans.append(msg)

    subscriptions = [
        node.create_subscription(
            OccupancyGrid, '/map', maps.append, map_qos
        ),
        node.create_subscription(
            LaserScan, '/scan', receive_scan, qos_profile_sensor_data
        ),
    ]

    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)

    try:
        deadline = time.monotonic() + 20.0
        tf_ready = False

        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            if maps and scans:
                scan = scans[0]
                tf_ready = tf_buffer.can_transform(
                    'map',
                    scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                )
                if tf_ready:
                    break

        assert maps, '20초 안에 /map을 받지 못했습니다.'
        assert scans, '20초 안에 /scan을 받지 못했습니다.'
        assert scans[0].header.frame_id == 'laser_frame'
        assert tf_ready, '스캔 시각의 map → laser_frame TF가 없습니다.'

        grid = maps[-1]
        assert grid.header.frame_id == 'map'
        assert grid.info.resolution > 0.0
        assert grid.info.width > 0
        assert grid.info.height > 0
        assert len(grid.data) == grid.info.width * grid.info.height
        assert all(-1 <= value <= 100 for value in grid.data)
        assert any(value >= 0 for value in grid.data), (
            '지도의 모든 칸이 미관측 상태입니다.'
        )

    finally:
        tf_listener.unregister()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
