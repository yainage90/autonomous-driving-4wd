"""Test doubles for tests that isolate navigation from the actuator receiver."""
from unittest.mock import Mock
import pytest


@pytest.fixture
def selection_actuator(monkeypatch):
    class SelectionActuator:
        period = 0.05
        def __init__(self, node, callback):
            self.ready = Mock(return_value=True)
            self.send = Mock()
            self.check = Mock()
            self.block = Mock()
        async def arm(self, epoch):
            pass
    monkeypatch.setattr('vehicle_control.command_guard.ActuatorLink', SelectionActuator)
