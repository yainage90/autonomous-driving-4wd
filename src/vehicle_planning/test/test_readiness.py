import math
import pytest

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from vehicle_planning.readiness import is_data_fresh, is_scan_valid, is_map_valid, is_odom_valid


@pytest.mark.parametrize(
    'received_at, stamp_ns, now_ros_ns, expected',
    [
        pytest.param(
            None, None, 10_000_000_000, False,
            id='not-received',
        ),
        pytest.param(
            99.9, 9_900_000_000, 10_000_000_000, True,
            id='fresh',
        ),
        pytest.param(
            99.5, 9_500_000_000, 10_000_000_000, True,
            id='at-timeout-boundary',
        ),
        pytest.param(
            99.4, 10_000_000_000, 10_000_000_000, False,
            id='interrupted-even-with-paused-ros-clock',
        ),
        pytest.param(
            99.9, 9_400_000_000, 10_000_000_000, False,
            id='recently-received-old-measurement',
        ),
        pytest.param(
            99.9, 10_050_000_000, 10_000_000_000, True,
            id='within-future-tolerance',
        ),
        pytest.param(
            99.9, 10_200_000_000, 10_000_000_000, False,
            id='too-far-in-future',
        ),
        pytest.param(
            99.9, 0, 0, False,
            id='ros-clock-not-started',
        ),
    ],
)
def test_is_data_fresh(
    received_at, stamp_ns, now_ros_ns, expected,
):
    assert is_data_fresh(
        received_at=received_at,
        stamp_ns=stamp_ns,
        now_monotonic=100.0,
        now_ros_ns=now_ros_ns,
    ) is expected


@pytest.mark.parametrize(
    'ranges, frame_id, expected',
    [
        pytest.param(
            [1.0, math.inf, 2.0], 'laser_frame', True,
            id='valid-scan',
        ),
        pytest.param(
            [math.inf] * 3, 'laser_frame', True,
            id='no-obstacle-returns',
        ),
        pytest.param(
            [], 'laser_frame', False,
            id='empty',
        ),
        pytest.param(
            [1.0, math.nan, 2.0], 'laser_frame', False,
            id='nan',
        ),
        pytest.param(
            [1.0, -math.inf, 2.0], 'laser_frame', False,
            id='negative-infinity',
        ),
        pytest.param(
            [0.0, 1.0, 2.0], 'laser_frame', False,
            id='below-minimum-range',
        ),
        pytest.param(
            [1.0, 11.0, 2.0], 'laser_frame', False,
            id='above-maximum-range',
        ),
        pytest.param(
            [1.0, 2.0], 'laser_frame', False,
            id='inconsistent-angle-and-count',
        ),
        pytest.param(
            [1.0, 2.0, 3.0], 'odom', False,
            id='wrong-frame',
        ),
    ],
)
def test_is_scan_valid(ranges, frame_id, expected):
    msg = LaserScan()
    msg.header.frame_id = frame_id
    msg.angle_min = -0.5
    msg.angle_max = 0.5
    msg.angle_increment = 0.5
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = ranges

    assert is_scan_valid(msg) is expected


@pytest.mark.parametrize(
    'resolution, width, data, expected',
    [
        pytest.param(
            0.05, 1, [0], True, id='valid-map',
        ),
        pytest.param(
            0.05, 1, [-1], True, id='unknown-cell-is-valid',
        ),
        pytest.param(
            math.nan, 1, [0], False, id='invalid-resolution',
        ),
        pytest.param(
            0.05, 0, [], False, id='empty-map',
        ),
        pytest.param(
            0.05, 2, [0], False, id='wrong-cell-count',
        ),
        pytest.param(
            0.05, 1, [101], False, id='invalid-cell-value',
        ),
    ],
)
def test_is_map_valid(resolution, width, data, expected):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = 1
    msg.info.origin.orientation.w = 1.0
    msg.data = data

    assert is_map_valid(msg) is expected


@pytest.mark.parametrize(
    'orientation_w, linear_x, angular_z, expected',
    [
        pytest.param(
            1.0, 0.1, -0.3, True, id='moving-is-valid',
        ),
        pytest.param(
            0.0, 0.0, 0.0, False, id='zero-quaternion',
        ),
        pytest.param(
            2.0, 0.0, 0.0, False, id='unnormalized-quaternion',
        ),
        pytest.param(
            1.0, math.nan, 0.0, False, id='invalid-linear-speed',
        ),
        pytest.param(
            1.0, 0.0, math.inf, False, id='invalid-angular-speed',
        ),
    ],
)
def test_is_odom_valid(
    orientation_w, linear_x, angular_z, expected,
):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.pose.pose.orientation.w = orientation_w
    msg.twist.twist.linear.x = linear_x
    msg.twist.twist.angular.z = angular_z

    assert is_odom_valid(msg) is expected
