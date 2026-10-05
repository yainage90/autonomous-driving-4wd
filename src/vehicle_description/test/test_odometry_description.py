from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


@pytest.fixture(scope='module')
def robot():
    """Xacro를 처리한 실제 URDF를 테스트에 제공한다."""
    package_dir = Path(__file__).resolve().parents[1]
    xacro_file = package_dir / 'urdf' / 'vehicle.urdf.xacro'

    document = xacro.process_file(str(xacro_file))
    return ET.fromstring(document.toxml())


def test_effective_wheel_separation_preserves_geometry(robot, vehicle_config):
    """계산용 윤거 보정이 실제 바퀴 배치를 변경하지 않아야 한다."""
    physical_separation = (
        vehicle_config['geometry']['wheel_separation_m']
    )
    multiplier = (
        vehicle_config['calibration']['wheel_separation_multiplier']
    )
    effective_separation = physical_separation * multiplier

    for axle in ('front', 'rear'):
        wheel_y = {}

        for side in ('left', 'right'):
            joint_name = f'{axle}_{side}_wheel_joint'
            origin = robot.find(
                f"./joint[@name='{joint_name}']/origin"
            )
            assert origin is not None, (
                f'{joint_name}의 origin이 없습니다.'
            )

            xyz_text = origin.get('xyz')
            assert xyz_text is not None, (
                f'{joint_name}의 xyz가 없습니다.'
            )

            xyz = [float(value) for value in xyz_text.split()]
            assert len(xyz) == 3
            wheel_y[side] = xyz[1]

        # 좌우 위치도 검사하여 바퀴가 뒤바뀌는 오류를 발견한다.
        assert wheel_y['left'] == pytest.approx(
            physical_separation / 2
        )
        assert wheel_y['right'] == pytest.approx(
            -physical_separation / 2
        )

        actual_separation = wheel_y['left'] - wheel_y['right']
        assert actual_separation == pytest.approx(
            physical_separation
        )

    diff_drive = robot.find(
        "./gazebo/plugin[@name='gz::sim::systems::DiffDrive']"
    )
    assert diff_drive is not None, 'DiffDrive 플러그인이 없습니다.'

    separation_text = diff_drive.findtext('wheel_separation')
    assert separation_text is not None, (
        'DiffDrive의 wheel_separation이 없습니다.'
    )

    assert float(separation_text) == pytest.approx(
        effective_separation
    )


def test_diff_drive_odometry_contract(robot):
    """오도메트리와 TF가 프로젝트의 토픽·frame 규약을 유지해야 한다."""
    plugins = robot.findall(
        "./gazebo/plugin[@name='gz::sim::systems::DiffDrive']"
    )
    assert len(plugins) == 1, (
        'DiffDrive 플러그인은 하나여야 합니다.'
    )
    diff_drive = plugins[0]

    expected_fields = {
        'topic': '/cmd_vel',
        'odom_topic': '/odom',
        'tf_topic': '/tf',
        'frame_id': 'odom',
        'child_frame_id': 'base_footprint',
    }

    for field, expected in expected_fields.items():
        assert diff_drive.findtext(field) == expected, (
            f'DiffDrive의 {field} 설정이 규약과 다릅니다.'
        )

    for frame in ('base_footprint', 'base_link'):
        assert robot.find(f"./link[@name='{frame}']") is not None, (
            f'{frame} 링크가 없습니다.'
        )
