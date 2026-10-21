"""A detached child must stop, and a different partition must remain untouched."""
import os
from pathlib import Path
import subprocess
import sys
from uuid import uuid4

from vehicle_simulation.process_cleanup import _owned_pids, cleanup_partition, stop_system


def test_cleanup_handles_detached_children_and_preserves_other_partitions():
    partition = 'cleanup_' + uuid4().hex
    owned_env = {**os.environ, 'GZ_PARTITION': partition}
    other_env = {**os.environ, 'GZ_PARTITION': 'other_' + uuid4().hex}
    sleeper = 'import time; time.sleep(60)'
    parent_script = (
        'import subprocess,sys,time; '
        f'child=subprocess.Popen([sys.executable,"-c",{sleeper!r}],start_new_session=True); '
        'print(child.pid,flush=True); time.sleep(60)'
    )
    other = subprocess.Popen([sys.executable, '-c', sleeper], env=other_env)
    parent = subprocess.Popen([sys.executable, '-c', parent_script], env=owned_env,
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              text=True, start_new_session=True)
    try:
        child = int(parent.stdout.readline())
        assert child in _owned_pids(partition)
        stop_system(parent, partition)
        assert parent.poll() is not None
        assert not _owned_pids(partition)
        assert other.poll() is None
    finally:
        cleanup_partition(partition)
        if parent.poll() is None:
            parent.kill()
        parent.wait(timeout=5)
        other.terminate()
        other.wait(timeout=5)
