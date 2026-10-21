"""Bounded cleanup for test systems, including detached Gazebo server children."""
import os
from pathlib import Path
import signal
import subprocess
import time


def _owned_pids(partition, command_names=None):
    if not partition:
        raise ValueError('A unique test Gazebo partition is required for process cleanup')
    marker = b'GZ_PARTITION=' + partition.encode()
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if command_names is not None and (entry / 'comm').read_text().strip() not in command_names:
                continue
            if marker in (entry / 'environ').read_bytes().split(b'\0'):
                result.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return result


def cleanup_partition(partition, command_names=None):
    """Signal only processes carrying the exact, caller-generated test partition."""
    for sig, grace in ((signal.SIGTERM, 3.), (signal.SIGKILL, 1.)):
        owned = _owned_pids(partition, command_names)
        for pid in owned:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        end = time.monotonic() + grace
        while time.monotonic() < end:
            if not _owned_pids(partition, command_names):
                return
            time.sleep(.05)
    remaining = _owned_pids(partition, command_names)
    if remaining:
        raise RuntimeError(f'Test processes did not terminate: {remaining}')


def stop_gazebo(partition, environment=None):
    """Stop the server through its own partition even when it detached from launch."""
    if not partition:
        raise ValueError('A unique test Gazebo partition is required')
    environment = {**(os.environ if environment is None else environment), 'GZ_PARTITION': partition}
    try:
        result = subprocess.run([
            'gz', 'service', '-s', '/server_control',
            '--reqtype', 'gz.msgs.ServerControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1500', '--req', 'stop: true',
        ], env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=4.)
        return result.returncode == 0 and 'data: true' in result.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def stop_system(process, partition, *, simulation=False, environment=None):
    try:
        if simulation:
            stop_gazebo(partition, environment)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
    finally:
        # gz sim may reparent the server outside the launch process group.
        cleanup_partition(partition)
