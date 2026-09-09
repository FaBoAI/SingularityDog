#!/usr/bin/env python3
"""Audit URDF topology, finite inertial properties and joint definitions; no meshes loaded."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def scalar(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('Non-finite numeric input')
    return result


def vector(value):
    values = [scalar(x) for x in value.split()]
    if len(values) != 3:
        raise ValueError('Expected a three-component vector')
    return values


def determinant(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def check_inertia(element):
    a, b, c, d, e, f = [scalar(element.attrib[k]) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')]
    scale = max(abs(x) for x in (a, b, c, d, e, f))
    if scale == 0:
        raise ValueError('Zero inertia')
    matrix = [[a / scale, d / scale, e / scale], [d / scale, b / scale, f / scale], [e / scale, f / scale, c / scale]]
    if not (matrix[0][0] > 0 and matrix[0][0] * matrix[1][1] - matrix[0][1] ** 2 > 0 and determinant(matrix) > 0):
        raise ValueError('Inertia must be positive definite')
    # A physical inertia also obeys the principal-moment triangle inequality.
    half_trace = sum(matrix[i][i] for i in range(3)) / 2
    covariance = [[(half_trace if i == j else 0) - matrix[i][j] for j in range(3)] for i in range(3)]
    minors = [covariance[i][i] for i in range(3)]
    minors += [covariance[i][i] * covariance[j][j] - covariance[i][j] ** 2 for i, j in ((0, 1), (0, 2), (1, 2))]
    minors.append(determinant(covariance))
    if min(minors) < -1e-10:
        raise ValueError('Principal inertia triangle inequality is violated')


def audit(path, expected_actuated=None):
    data = Path(path).read_bytes()
    if b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper():
        raise ValueError('External/entity declarations are unsupported')
    robot = ET.fromstring(data)
    if robot.tag != 'robot':
        raise ValueError('Expected robot element')
    links = robot.findall('link')
    names = [x.attrib['name'] for x in links]
    if not names or len(set(names)) != len(names):
        raise ValueError('Missing or duplicate links')
    total = 0.0
    for link in links:
        inertia = link.find('inertial')
        if inertia is None or inertia.find('mass') is None or inertia.find('inertia') is None:
            raise ValueError('Every link requires explicit inertial data: ' + link.attrib['name'])
        mass = scalar(inertia.find('mass').attrib['value'])
        if mass <= 0:
            raise ValueError('Every link must have positive mass')
        total += mass
        check_inertia(inertia.find('inertia'))
        for origin in link.findall('.//origin'):
            vector(origin.get('xyz', '0 0 0'))
            vector(origin.get('rpy', '0 0 0'))
    parents = {}
    joints = set()
    moving = 0
    for joint in robot.findall('joint'):
        name = joint.attrib['name']
        if name in joints:
            raise ValueError('Duplicate joint')
        joints.add(name)
        kind = joint.attrib['type']
        if kind not in ('fixed', 'revolute', 'continuous', 'prismatic') or joint.find('mimic') is not None:
            raise ValueError('Unsupported joint type or mimic')
        parent, child = joint.find('parent').attrib['link'], joint.find('child').attrib['link']
        if parent not in names or child not in names or parent == child or child in parents:
            raise ValueError('Invalid link tree')
        parents[child] = parent
        origin = joint.find('origin')
        if origin is not None:
            vector(origin.get('xyz', '0 0 0'))
            vector(origin.get('rpy', '0 0 0'))
        if kind != 'fixed':
            moving += 1
            axis = joint.find('axis')
            values = vector(axis.get('xyz', '1 0 0') if axis is not None else '1 0 0')
            if abs(sum(x * x for x in values) - 1) > 1e-6:
                raise ValueError('Joint axis must be normalized')
            limit = joint.find('limit')
            if limit is None or scalar(limit.get('effort', '0')) <= 0 or scalar(limit.get('velocity', '0')) <= 0:
                raise ValueError('Positive explicit effort and velocity limits required')
            if kind != 'continuous' and scalar(limit.attrib['lower']) >= scalar(limit.attrib['upper']):
                raise ValueError('Joint bounds are inverted')
    roots = set(names) - set(parents)
    if len(roots) != 1:
        raise ValueError('Expected one root')
    for name in names:
        seen = set()
        cursor = name
        while cursor in parents:
            if cursor in seen:
                raise ValueError('Cyclic tree')
            seen.add(cursor)
            cursor = parents[cursor]
    if expected_actuated is not None and moving != expected_actuated:
        raise ValueError('Unexpected actuated joint count')
    return {'schema': 'singularitydog.urdf-audit.v1', 'status': 'CPU_STRUCTURE_PASS', 'links': len(names), 'joints': len(joints),
            'actuated_joints': moving, 'total_declared_mass_kg': total,
            'limits': ['No mesh, collision, global center-of-mass, strength, simulator or walking validation.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--expected-actuated', type=int)
    args = parser.parse_args()
    print(json.dumps(audit(args.urdf, args.expected_actuated), indent=2))


if __name__ == '__main__':
    main()
