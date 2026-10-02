#!/usr/bin/env python3
"""Split the source ASCII STL into ROS-aligned binary visual meshes (stdlib only)."""

import argparse
from array import array
from collections import defaultdict
from math import sqrt
from pathlib import Path
import struct


PACKAGE = Path(__file__).resolve().parents[1]
AXLE_MIDPOINT = (0.0, 0.03453, -0.011)


def read_components(path):
    """Weld to 10 nm for connectivity only; retain original triangle coordinates."""
    coordinates = array('d')
    vertex_ids = {}
    parents = []
    triangles = []
    pending = []

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    with path.open() as source:
        for line in source:
            fields = line.split()
            if not fields or fields[0] != 'vertex':
                continue
            point = tuple(float(value) for value in fields[1:])
            if len(point) != 3:
                raise ValueError('Expected three coordinates per vertex')
            coordinates.extend(point)
            key = tuple(round(value, 8) for value in point)
            if key not in vertex_ids:
                vertex_ids[key] = len(parents)
                parents.append(len(parents))
            pending.append(vertex_ids[key])
            if len(pending) == 3:
                first = root(pending[0])
                for index in pending[1:]:
                    parents[root(index)] = first
                triangles.append(pending[0])
                pending = []
    if pending or not triangles:
        raise ValueError('Expected complete triangles in an ASCII STL')

    groups = defaultdict(list)
    for index, vertex in enumerate(triangles):
        groups[root(vertex)].append(index)
    return coordinates, list(groups.values())


def bounds(coordinates, triangles):
    lower = [float('inf')] * 3
    upper = [float('-inf')] * 3
    for triangle in triangles:
        for offset in range(triangle * 9, triangle * 9 + 9, 3):
            for axis in range(3):
                value = coordinates[offset + axis]
                lower[axis] = min(lower[axis], value)
                upper[axis] = max(upper[axis], value)
    return lower, upper


def write_mesh(path, coordinates, triangles, origin):
    # Proper rotation: ROS (x, y, z) = STL (-z, -x, y), followed by translation.
    with path.open('wb') as output:
        output.write(b'ROS-aligned visual mesh; coordinates in meters'.ljust(80, b'\0'))
        output.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            points = []
            for offset in range(triangle * 9, triangle * 9 + 9, 3):
                x, y, z = coordinates[offset:offset + 3]
                points.append((-(z - origin[2]), -(x - origin[0]), y - origin[1]))
            a = [points[1][i] - points[0][i] for i in range(3)]
            b = [points[2][i] - points[0][i] for i in range(3)]
            normal = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
                      a[0]*b[1] - a[1]*b[0]]
            length = sqrt(sum(value**2 for value in normal))
            if length:
                normal = [value / length for value in normal]
            output.write(struct.pack('<12fH', *normal,
                                     *(value for point in points for value in point), 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=PACKAGE / 'meshes/four-wheel-drvie-main-body.stl')
    parser.add_argument('--output-dir', type=Path, default=PACKAGE / 'meshes')
    args = parser.parse_args()
    coordinates, groups = read_components(args.source)
    parts = []
    wheels = {}
    for triangles in groups:
        lower, upper = bounds(coordinates, triangles)
        size = [upper[i] - lower[i] for i in range(3)]
        center = [(upper[i] + lower[i]) / 2 for i in range(3)]
        parts.append((triangles, lower, upper, center))
        if (abs(size[0] - 0.027) < 0.0001
                and all(abs(size[i] - 0.067) < 0.0001 for i in (1, 2))):
            name = ('front' if center[2] < AXLE_MIDPOINT[2] else 'rear')
            name += '_left' if center[0] < 0 else '_right'
            if name in wheels:
                raise ValueError(f'Duplicate wheel: {name}')
            wheels[name] = (center, triangles)
    if len(wheels) != 4:
        raise ValueError(f'Expected four 67 mm wheels, found {len(wheels)}')

    output_parts = {'chassis_body': []}
    for name in wheels:
        output_parts[name + '_wheel'] = []
    for triangles, lower, upper, center in parts:
        owner = 'chassis_body'
        for name, (wheel_center, tire) in wheels.items():
            # Wheel, hub, axle-end screw and hub clamp screw rotate together.
            # Motor bodies cross the inner 41 mm boundary and stay on the chassis.
            same_side = center[0] * wheel_center[0] > 0
            outside = lower[0] > 0.041 or upper[0] < -0.041
            near_axle = (abs(center[1] - wheel_center[1]) < 0.010
                         and abs(center[2] - wheel_center[2]) < 0.015)
            if triangles is tire or (same_side and outside and near_axle):
                owner = name + '_wheel'
                break
        output_parts[owner].extend(triangles)

    assert sum(map(len, output_parts.values())) == len(coordinates) // 9
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, triangles in output_parts.items():
        origin = AXLE_MIDPOINT if name == 'chassis_body' else wheels[name[:-6]][0]
        destination = args.output_dir / (name + '.stl')
        if destination.resolve() == args.source.resolve():
            raise ValueError('Output must not overwrite source STL')
        write_mesh(destination, coordinates, triangles, origin)
        print(f'{destination.name}: {len(triangles)} triangles, '
              f'{destination.stat().st_size:,} bytes')


if __name__ == '__main__':
    main()
