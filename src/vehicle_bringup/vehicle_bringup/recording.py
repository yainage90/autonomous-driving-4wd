import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
import argparse
import math
import os
import shlex

def snapshot_inputs(workspace: Path, run_dir: Path) -> dict:
    workspace = workspace.resolve()
    run_dir = run_dir.resolve()
    source = workspace / 'src'

    # src 안에 출력하면 복사 대상에 출력물이 다시 포함된다.
    if run_dir.is_relative_to(source):
        raise ValueError('Run directory must be outside src')

    def git_output(*args: str) -> str:
        return subprocess.check_output(
            ['git', '-C', str(workspace), *args],
            text=True,
        ).rstrip('\n')

    git_head = git_output('rev-parse', 'HEAD')
    git_status = git_output('status', '--porcelain=v1')

    # 같은 실행 디렉터리를 재사용하여 자료를 덮어쓰지 않는다.
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / 'COLCON_IGNORE').touch()
    snapshot = run_dir / 'inputs' / 'src'
    shutil.copytree(source, snapshot)

    hashes = {}
    for path in sorted(snapshot.rglob('*')):
        if path.is_file():
            relative = path.relative_to(run_dir).as_posix()
            hashes[relative] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()

    manifest = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'git_head': git_head,
        'git_status': git_status,
        'source_sha256': hashes,
    }
    (run_dir / 'source_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    return manifest



TOPICS = [
    '/clock',
    '/scan',
    '/odom',
    '/ground_truth/odom',
    '/tf',
    '/tf_static',
    '/cmd_vel_manual',
    '/cmd_vel_auto',
    '/cmd_vel',
    '/actuator/command',
    '/actuator/status',
    '/navigation_allowed',
    '/control_state',
    '/navigation_permission',
    '/navigation_command',
    '/navigation_manager/events',
    '/rosout',
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=Path.cwd())
    parser.add_argument(
        '--run-dir', type=Path,
        help='Output directory (default: <workspace>/evaluation_runs/run_<UTC timestamp>)',
    )
    parser.add_argument('--scenario', required=True)
    parser.add_argument(
        '--mode', choices=['navigation', 'manual'], default='navigation',
    )
    parser.add_argument('--goal', type=float, nargs=3)
    parser.add_argument('--start-map', type=float, nargs=3)
    args = parser.parse_args()

    if args.mode == 'navigation' and args.goal is None:
        parser.error('navigation mode requires --goal x y yaw')

    for values in (args.goal, args.start_map):
        if values is not None and not all(map(math.isfinite, values)):
            parser.error('Pose values must be finite')

    workspace = args.workspace.resolve()
    if args.run_dir is None:
        run_name = datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%fZ')
        run_dir = workspace / 'evaluation_runs' / run_name
    else:
        run_dir = args.run_dir.resolve()
    snapshot_inputs(workspace, run_dir)
    source = run_dir / 'inputs' / 'src'

    inputs = {
        'world': 'vehicle_simulation/worlds/evaluation.sdf',
        'map': 'vehicle_localization/maps/evaluation.yaml',
        'safety_params_file': 'vehicle_bringup/config/safety.yaml',
        'amcl_params_file': 'vehicle_localization/config/amcl.yaml',
        'nav2_params_file': 'vehicle_planning/config/nav2.yaml',
        'controller_params_file': 'vehicle_control/config/controller.yaml',
    }
    launch_command = [
        'ros2', 'launch', 'vehicle_bringup', 'system.launch.py',
        f'mode:={args.mode}',
    ]
    launch_command.extend(
        f'{name}:={source / relative}'
        for name, relative in inputs.items()
    )

    bag_command = [
        'ros2', 'bag', 'record',
        '-s', 'mcap',
        '-o', str(run_dir / 'bag'),
        '--qos-profile-overrides-path',
        str(source / 'vehicle_simulation/config/recording_qos.yaml'),
        '--topics', *TOPICS,
    ]

    metadata = {
        'schema_version': 1,
        'scenario': args.scenario,
        'mode': args.mode,
        'planned_goal_map': args.goal,
        'expected_start_map': args.start_map,
        'launch_command': launch_command,
        'bag_command': bag_command,
        'bag_timestamp_source': 'system_time',
    }
    (run_dir / 'run.json').write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    (run_dir / 'start_system.sh').write_text(
        '#!/usr/bin/env bash\n'
        + 'exec ' + shlex.join(launch_command) + '\n',
        encoding='utf-8',
    )

    print(f'Run directory: {run_dir}', flush=True)
    print(
        'Start the system in another terminal:\n'
        + shlex.join(['bash', str(run_dir / 'start_system.sh')]),
        flush=True,
    )
    os.execvp(bag_command[0], bag_command)
