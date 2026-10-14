"""Pendant simulator: sends the ESP32's frames at 50 Hz through a virtual serial port.

    ros2 run movemaster_teleop pendant_sim
    ros2 run movemaster_teleop jog_node --ros-args -p port:=<the /dev/pts/N it prints>

Commands: "sel 1 2" (X joint, Y joint), "x 300", "y -500" (-1000..1000), "0" (release), "q".
"""

import os
import pty
import threading
import time
import tty

from movemaster_teleop.pendant_frame import FULL_SCALE, PendantFrame, SYNC


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    state = {'x_joint': 1, 'y_joint': 2, 'x': 0, 'y': 0}
    running = True

    def send():
        seq = 0
        while running:
            frame = PendantFrame(seq=seq, manual=True, **state)
            os.write(master, bytes([SYNC]) + frame.encode())
            seq = (seq + 1) & 0xFF
            time.sleep(0.02)

    threading.Thread(target=send, daemon=True).start()
    print(f'Pendant on {os.ttyname(slave)}. Selected X joint_1, Y joint_2.')
    print('Commands: sel X Y | x VALUE | y VALUE | 0 | q')
    try:
        while True:
            words = input('> ').split()
            if not words:
                continue
            if words == ['q']:
                break
            try:
                if words == ['0']:
                    state.update(x=0, y=0)
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
