import math

import pytest
from geometry_msgs.msg import Twist
from vehicle_interfaces.msg import ActuatorCommand
from vehicle_interfaces.srv import ArmActuator
from vehicle_drivers.mock_hardware import MockReceiver, ScanMap
from ament_index_python.packages import get_package_share_directory


def activate(receiver, now=1.):
    status = receiver.heartbeat(now)
    request = ArmActuator.Request(actuator_session=status.actuator_session,
        generation=status.generation, heartbeat=status.heartbeat,
        guard_session='guard', operation=ArmActuator.Request.ARM)
    response = receiver.arm(request, now)
    assert response.success
    status = receiver.heartbeat(now + .01)
    velocity = Twist()
    velocity.linear.x = .1
    return ActuatorCommand(actuator_session=status.actuator_session,
        generation=status.generation, guard_session='guard', sequence=1,
        heartbeat=status.heartbeat, velocity=velocity)


def test_receiver_expiry_rejects_replayed_command_and_requires_new_arm():
    receiver = MockReceiver()
    command = activate(receiver)
    assert receiver.command(command, 1.02)
    receiver.heartbeat(1.3)
    receiver.tick(1.33)
    assert not receiver.status.armed
    assert receiver.linear == 0.
    assert not receiver.command(command, 1.34)
    new_command = activate(receiver, 2.)
    assert receiver.command(new_command, 2.02)


@pytest.mark.parametrize('field,value', [
    ('actuator_session', 'old'), ('generation', 0), ('guard_session', 'other'),
    ('heartbeat', 0), ('sequence', 0),
])
def test_receiver_rejects_invalid_token(field, value):
    receiver = MockReceiver()
    command = activate(receiver)
    setattr(command, field, value)
    assert not receiver.command(command, 1.02)
    assert receiver.linear == 0.


def test_receiver_rejects_duplicate_and_invalid_velocity():
    receiver = MockReceiver()
    command = activate(receiver)
    assert receiver.command(command, 1.02)
    assert not receiver.command(command, 1.03)
    command.sequence = 2
    command.velocity.linear.x = math.nan
    assert not receiver.command(command, 1.04)
    assert not receiver.status.armed
    assert receiver.linear == 0.


def test_mock_scan_uses_installed_map_and_has_finite_wall_returns():
    model = ScanMap(f"{get_package_share_directory('vehicle_localization')}/maps/evaluation.yaml")
    ranges = model.ranges(.03, 0., 0.)
    assert len(ranges) == 360
    assert all(value == math.inf or .1 <= value <= 12. for value in ranges)
    assert sum(math.isfinite(value) for value in ranges) > 300
