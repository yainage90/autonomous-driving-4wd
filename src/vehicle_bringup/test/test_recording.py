import hashlib
import json
from unittest.mock import Mock

import pytest

from vehicle_bringup.recording import snapshot_inputs
from vehicle_bringup import recording


def test_snapshot_preserves_inputs_and_rejects_reuse(
    tmp_path, monkeypatch,
):
    workspace = tmp_path / 'workspace'
    source_file = workspace / 'src' / 'example' / 'params.yaml'
    source_file.parent.mkdir(parents=True)
    original = b'max_speed: 0.2\n'
    source_file.write_bytes(original)

    def fake_git_output(command, **kwargs):
        responses = {
            ('rev-parse', 'HEAD'): 'abc123\n',
            ('status', '--porcelain=v1'):
                ' M src/example/params.yaml\n',
        }
        return responses[tuple(command[3:])]

    monkeypatch.setattr(
        'vehicle_bringup.recording.subprocess.check_output',
        Mock(side_effect=fake_git_output),
    )

    run_dir = tmp_path / 'runs' / 's01_r1'

    snapshot_inputs(workspace, run_dir)
    source_file.write_text('max_speed: 0.5\n', encoding='utf-8')

    saved = run_dir / 'inputs/src/example/params.yaml'
    manifest = json.loads(
        (run_dir / 'source_manifest.json').read_text(encoding='utf-8')
    )

    assert saved.read_bytes() == original
    assert manifest['git_head'] == 'abc123'
    assert manifest['source_sha256'][
        'inputs/src/example/params.yaml'
    ] == hashlib.sha256(original).hexdigest()

    with pytest.raises(FileExistsError):
        snapshot_inputs(workspace, run_dir)

    assert saved.read_bytes() == original


@pytest.mark.parametrize('explicit_path', [False, True])
def test_recording_output_directory(tmp_path, monkeypatch, explicit_path):
    workspace = tmp_path / 'workspace'
    workspace.mkdir()

    def prepare_snapshot(workspace, run_dir):
        run_dir.mkdir(parents=True, exist_ok=False)

    snapshot = Mock(side_effect=prepare_snapshot)
    monkeypatch.setattr(recording, 'snapshot_inputs', snapshot)
    recorder = Mock()
    monkeypatch.setattr(recording.os, 'execvp', recorder)
    argv = [
        'record_run', '--workspace', str(workspace),
        '--scenario', 's01', '--goal', '1', '0', '0',
    ]
    custom_dir = tmp_path / 'custom_run'
    if explicit_path:
        argv.extend(['--run-dir', str(custom_dir)])
    monkeypatch.setattr('sys.argv', argv)

    recording.main()

    saved_workspace, run_dir = snapshot.call_args.args
    assert saved_workspace == workspace
    if explicit_path:
        assert run_dir == custom_dir
    else:
        assert run_dir.parent == workspace / 'evaluation_runs'
        assert run_dir.name.startswith('run_')

    metadata = json.loads((run_dir / 'run.json').read_text(encoding='utf-8'))
    assert metadata['backend'] == 'simulation'
    assert 'backend:=simulation' in metadata['launch_command']
    for argument in ('lidar_config_file', 'simulation_config_file'):
        assert any(item.startswith(argument + ':=') for item in metadata['launch_command'])
    command = metadata['bag_command']
    assert command[command.index('-o') + 1] == str(run_dir / 'bag')
    assert str(run_dir / 'inputs' / 'src') in (
        run_dir / 'start_system.sh'
    ).read_text(encoding='utf-8')
    recorder.assert_called_once_with('ros2', command)
