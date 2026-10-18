import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry


def is_data_fresh(
    *,
    received_at: float | None,
    stamp_ns: int | None,
    now_monotonic: float,
    now_ros_ns: int,
    timeout_sec: float = 0.5,
    future_tolerance_sec: float = 0.1,
) -> bool:
    if received_at is None or stamp_ns is None:
        return False

    if stamp_ns <= 0 or now_ros_ns <= 0:
        return False

    received_age = now_monotonic - received_at
    measurement_age = (now_ros_ns - stamp_ns) / 1_000_000_000

    return (
        0.0 <= received_age <= timeout_sec
        and -future_tolerance_sec <= measurement_age <= timeout_sec
    )


def is_scan_valid(msg: LaserScan) -> bool:
    if msg.header.frame_id != 'laser_frame':
        return False

    metadata = (
        msg.angle_min,
        msg.angle_max,
        msg.angle_increment,
        msg.range_min,
        msg.range_max,
    )
    if not all(math.isfinite(value) for value in metadata):
        return False

    if (
        len(msg.ranges) < 2
        or msg.angle_increment <= 0.0
        or msg.angle_max <= msg.angle_min
        or msg.range_min < 0.0
        or msg.range_max <= msg.range_min
    ):
        return False

    last_angle = (
        msg.angle_min
        + (len(msg.ranges) - 1) * msg.angle_increment
    )
    if not math.isclose(
        last_angle,
        msg.angle_max,
        rel_tol=0.0,
        abs_tol=1e-5,
    ):
        return False

    return all(
        distance == math.inf
        or (
            math.isfinite(distance)
            and msg.range_min - 1e-5
            <= distance
            <= msg.range_max + 1e-5
        )
        for distance in msg.ranges
    )


def _is_pose_valid(pose: Pose) -> bool:
    p = pose.position
    q = pose.orientation

    if not all(math.isfinite(value) for value in (
        p.x, p.y, p.z,
        q.x, q.y, q.z, q.w,
    )):
        return False

    return math.isclose(
        math.hypot(q.x, q.y, q.z, q.w),
        1.0,
        rel_tol=0.0,
        abs_tol=1e-3,
    )


def is_map_valid(msg: OccupancyGrid) -> bool:
    info = msg.info

    if (
        msg.header.frame_id != 'map'
        or not math.isfinite(info.resolution)
        or info.resolution <= 0.0
        or info.width == 0
        or info.height == 0
        or len(msg.data) != info.width * info.height
        or not _is_pose_valid(info.origin)
    ):
        return False

    return all(-1 <= value <= 100 for value in msg.data)


def is_odom_valid(msg: Odometry) -> bool:
    if (
        msg.header.frame_id != 'odom'
        or msg.child_frame_id != 'base_footprint'
        or not _is_pose_valid(msg.pose.pose)
    ):
        return False

    twist = msg.twist.twist

    return all(math.isfinite(value) for value in (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    ))
