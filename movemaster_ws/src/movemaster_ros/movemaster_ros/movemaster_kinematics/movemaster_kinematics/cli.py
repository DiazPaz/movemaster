"""Kinematics from the terminal, in the HMI's units (mm and degrees), without the robot:

    ros2 run movemaster_kinematics kinematics fk 0 0 0 0 0
    ros2 run movemaster_kinematics kinematics ik 157.4 0 255 -90 0 [--near 10 0 0 0 0]

fk takes J1..J5 and prints the pose of the tcp; ik takes X Y Z PITCH ROLL and prints J1..J5, or
why the pose cannot be reached.
"""

import argparse
import math
import sys

from movemaster_kinematics.kinematics import Arm, Pose, Unreachable


def _shown(value, digits):
    """Rounded, without a minus sign on zero."""
    return f'{round(value, digits) + 0.0:.{digits}f}'


def describe_pose(pose):
    return (f'X {_shown(pose.x * 1000, 1)} mm  Y {_shown(pose.y * 1000, 1)} mm  '
            f'Z {_shown(pose.z * 1000, 1)} mm  pitch {_shown(math.degrees(pose.pitch), 1)} deg  '
            f'roll {_shown(math.degrees(pose.roll), 1)} deg')


def describe_joints(q):
    return '  '.join(f'J{i} {_shown(math.degrees(angle), 2)} deg' for i, angle in enumerate(q, 1))


def main(argv=None):
    parser = argparse.ArgumentParser(description='MoveMaster kinematics (mm and degrees).')
    parser.add_argument('--dh-config', help='dh.yaml (default: movemaster_description)')
    parser.add_argument('--joint-config', help='joints.json (default: movemaster_hardware)')
    commands = parser.add_subparsers(dest='command', required=True)
    fk = commands.add_parser('fk', help='pose of the tcp for J1..J5')
    fk.add_argument('joints', nargs=5, type=float, metavar='J')
    ik = commands.add_parser('ik', help='J1..J5 for a pose of the tcp')
    ik.add_argument('pose', nargs=5, type=float, metavar=('X', 'Y', 'Z', 'PITCH', 'ROLL'))
    ik.add_argument('--near', nargs=5, type=float, metavar='J',
                    help='where the joints are (default Home): the closest solution wins')
    args = parser.parse_args(argv)

    arm = Arm.from_files(args.dh_config, args.joint_config)
    if args.command == 'fk':
        print(describe_pose(arm.fk([math.radians(angle) for angle in args.joints])))
        return 0
    x, y, z, pitch, roll = args.pose
    pose = Pose(x / 1000, y / 1000, z / 1000, math.radians(pitch), math.radians(roll))
    near = [math.radians(angle) for angle in args.near] if args.near else None
    try:
        print(describe_joints(arm.ik(pose, near)))
    except Unreachable as error:
        print(f'Unreachable: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
