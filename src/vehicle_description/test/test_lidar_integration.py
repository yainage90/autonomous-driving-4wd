import math
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


@pytest.mark.skipif(
    os.environ.get('RUN_LIDAR_INTEGRATION') != '1',
    reason='실행 중인 Gazebo가 필요한 통합 테스트입니다.',
)
def test_scan_is_received_through_bridge(lidar_config):
    rclpy.init()
    node = rclpy.create_node('lidar_integration_test')

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    scans = []

    subscription = node.create_subscription(
        LaserScan,
        '/scan',
        scans.append,
        qos_profile_sensor_data,
    )

    try:
        deadline = time.monotonic() + 20.0

        while len(scans) < 3 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)

        assert len(scans) >= 3, (
            f'20초 안에 스캔 3개를 받지 못했습니다: {len(scans)}개'
        )

        timestamps = []

        for scan in scans[:3]:
            assert scan.header.frame_id == 'laser_frame'
            assert len(scan.ranges) == lidar_config['horizontal_samples']

            assert scan.angle_min == pytest.approx(
                lidar_config['min_angle_rad']
            )
            assert scan.angle_max == pytest.approx(
                lidar_config['max_angle_rad']
            )
            assert scan.angle_increment > 0.0
            assert (
                scan.angle_min
                + (len(scan.ranges) - 1) * scan.angle_increment
            ) == pytest.approx(scan.angle_max, abs=1e-5)

            assert scan.range_min == pytest.approx(
                lidar_config['min_range_m']
            )
            assert scan.range_max == pytest.approx(
                lidar_config['max_range_m']
            )

            assert any(math.isfinite(r) for r in scan.ranges)

            for distance in scan.ranges:
                assert not math.isnan(distance)

                if math.isfinite(distance):
                    assert (
                        scan.range_min - 1e-5
                        <= distance
                        <= scan.range_max + 1e-5
                    )
                else:
                    assert distance == math.inf

            stamp = scan.header.stamp
            timestamps.append(
                stamp.sec * 1_000_000_000 + stamp.nanosec
            )

        assert timestamps[0] < timestamps[1] < timestamps[2]

    finally:
        executor.shutdown()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()