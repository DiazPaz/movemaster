"""The bringup launch, what it includes and the systemd service."""

import importlib.util
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent


def load_launch():
    path = PACKAGE / 'launch' / 'bringup.launch.py'
    spec = importlib.util.spec_from_file_location('movemaster_bringup_launch', path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    return launch.generate_launch_description()


def test_arguments_and_defaults():
    from launch.actions import DeclareLaunchArgument
    arguments = {entity.name: entity for entity in load_launch().entities
                 if isinstance(entity, DeclareLaunchArgument)}
    assert set(arguments) == {'pendant', 'rviz', 'use_mock_hardware', 'joint_config',
                              'can_interface'}

    def default(name):
        return ''.join(s.text for s in arguments[name].default_value)
    # By default: the real arm, no pendant and no RViz, as the service on the Pi runs it.
    assert default('pendant') == ''
    assert default('rviz') == 'false'
    assert default('use_mock_hardware') == 'false'
    assert default('can_interface') == 'can0'


def test_included_files_exist():
    # The launch finds them in the installed packages; here, in the source tree.
    for path in ('movemaster_control/launch/movemaster_control.launch.py',
                 'movemaster_teleop/launch/jog.launch.py',
                 'movemaster_description/rviz/display.rviz',
                 'movemaster_hardware/config/joints.json'):
        assert (PACKAGES / path).is_file(), path


def test_service():
    settings = {}  # key -> every value: systemd allows a key several times
    for line in (PACKAGE / 'systemd' / 'movemaster.service').read_text().splitlines():
        if '=' in line and not line.startswith('#'):
            key, value = line.split('=', 1)
            settings.setdefault(key, []).append(value)
    assert any('ros2 launch movemaster_bringup bringup.launch.py' in v
               for v in settings['ExecStart'])
    assert any('type can bitrate 1000000' in v for v in settings['ExecStartPre'])
    assert settings['KillSignal'] == ['SIGINT']    # stops like Ctrl+C: the heartbeat ends
    assert settings['LimitRTPRIO'] == ['99']       # limits.conf does not apply to services
    # A restart after an emergency stop would take the power-up pose as the reference.
    assert settings['Restart'] == ['no']
