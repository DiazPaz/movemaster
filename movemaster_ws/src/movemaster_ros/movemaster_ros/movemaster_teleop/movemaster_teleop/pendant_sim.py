"""Pendant simulator: sends what the ESP32 sends, 50 times a second, through a virtual serial
port. Text by default, as hmi/esp32/main.cpp; --binary for the frames of firmware/.

    ros2 run movemaster_teleop pendant_sim [--binary]
    ros2 launch movemaster_teleop jog.launch.py port:=<the /dev/pts/N it prints>

Commands: "manual" or "auto" (the switch; starts in AUTO), "sel 4 1" (X joint, Y joint),
"x 300", "y -500" (-1000..1000), "0" (release), "grip" (the joystick button), "angles" (the
last angles jog_node sent), "q".
"""

import os
import pty
import select
import sys
import threading
import time
import tty

from movemaster_teleop import pendant_text
from movemaster_teleop.pendant_frame import FULL_SCALE, PendantFrame, SYNC


def main():
    binary = '--binary' in sys.argv[1:]
    master, slave = pty.openpty()
    tty.setraw(slave)
    state = {'manual': False, 'gripper': False, 'x_joint': 4, 'y_joint': 1, 'x': 0, 'y': 0}
    received = {'angles': None}
    running = True

    def send():
        seq = 0
        incoming = bytearray()
        while running:
            # As the ESP32: the joystick moves nothing outside MANUAL.
            moving = state if state['manual'] else {**state, 'x': 0, 'y': 0}
            frame = PendantFrame(seq=seq, **moving)
            os.write(master, bytes([SYNC]) + frame.encode() if binary
                     else pendant_text.encode(frame, received['angles']))
            seq = (seq + 1) & 0xFF
            # What jog_node writes back: the measured angles (text protocol only).
            while select.select([master], [], [], 0)[0]:
                incoming += os.read(master, 1024)
            *lines, rest = incoming.split(b'\n')
            incoming = bytearray(rest)
            for line in lines:
                if line.startswith(b'ANGLES,'):
                    try:
                        received['angles'] = [float(v) for v in line[7:].split(b',')]
                    except ValueError:
                        pass
            time.sleep(0.02)

    threading.Thread(target=send, daemon=True).start()
    print(f'Pendant on {os.ttyname(slave)} ({"binary" if binary else "text"}). Switch in AUTO; '
          'selected X joint_4, Y joint_1.')
    print('Commands: manual | auto | sel X Y | x VALUE | y VALUE | 0 | grip | angles | q')
    try:
        while True:
            words = input('> ').split()
            if not words:
                continue
            if words == ['q']:
                break
            try:
                if words in (['manual'], ['auto']):
                    state['manual'] = words == ['manual']
                elif words == ['0']:
                    state.update(x=0, y=0)
                elif words == ['grip']:
                    state['gripper'] = not state['gripper']
                elif words == ['angles']:
                    print(received['angles'])
                    continue
                elif len(words) == 3 and words[0] == 'sel':
                    state.update(x_joint=int(words[1]), y_joint=int(words[2]))
                elif len(words) == 2 and words[0] in ('x', 'y'):
                    state[words[0]] = max(-FULL_SCALE, min(FULL_SCALE, int(words[1])))
                else:
                    raise ValueError
            except ValueError:
                print('?')
                continue
            print(state)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        running = False


if __name__ == '__main__':
    main()
