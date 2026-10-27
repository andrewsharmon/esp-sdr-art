#!/usr/bin/env python3
"""Flash a Cheap Yellow Display and assign the slice of the band it shows.

Examples (activate the ESP-IDF environment first, for esptool and pyserial):

  # Board 3 of 4 side by side covering 2400-2480 MHz: 20 MHz span at 2450 MHz
  python tools/cyd_program.py --port /dev/cu.usbserial-140 --flash \
      --band 2400 2480 --boards 4 --index 2

  # Explicit centre and span, peak hold, without reflashing
  python tools/cyd_program.py --port /dev/cu.usbserial-140 --center 2437.5 --span 16

The board stores the slice in NVS (CYDSET) and returns to it after every power
cycle and after a host session. Spans must be ones the display offers.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import serial

SPANS_MHZ = (80, 60, 40, 20, 16, 10, 5, 2)
ROOT = Path(__file__).resolve().parents[1]


def flash(port, build_dir, baud):
    args = [sys.executable, '-m', 'esptool', '--chip', 'esp32', '-p', port, '-b', str(baud),
            '--before', 'default-reset', '--after', 'hard-reset', 'write-flash', '@flash_args']
    subprocess.run(args, cwd=build_dir, check=True)


def send(port, lines, baud=2000000):
    """Opening the CH340 port resets the board: wait for the firmware first."""
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, baud, 1, False, False
    s.open()
    try:
        time.sleep(3.5)
        s.reset_input_buffer()
        replies = []
        for line in lines:
            s.write((line + '\n').encode())
            # Drop any boot-log noise left from the reset-on-open.
            reply = s.readline().decode(errors='replace')
            replies.append(''.join(c for c in reply if c.isprintable()).strip().split('  ')[-1].strip())
        s.write(b'RELEASE\n')
        s.readline()
        return replies
    finally:
        s.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', required=True)
    p.add_argument('--flash', action='store_true', help='write build-cyd first')
    p.add_argument('--build-dir', type=Path, default=ROOT / 'build-cyd')
    p.add_argument('--flash-baud', type=int, default=460800)
    p.add_argument('--center', type=float, help='centre frequency in MHz (1 kHz resolution)')
    p.add_argument('--span', type=int, choices=SPANS_MHZ, help='span in MHz')
    p.add_argument('--band', type=float, nargs=2, metavar=('START', 'STOP'), help='band to split, MHz')
    p.add_argument('--boards', type=int, help='number of boards across --band')
    p.add_argument('--index', type=int, help='this board, 0 = lowest frequency')
    p.add_argument('--detector', choices=('pk', 'avg'))
    a = p.parse_args()

    if a.band:
        if a.boards is None or a.index is None or not 0 <= a.index < a.boards:
            p.error('--band needs --boards and --index (0 .. boards-1)')
        width = (a.band[1] - a.band[0]) / a.boards
        if round(width, 6) not in SPANS_MHZ:
            p.error(f'each board would show {width:g} MHz; choose a band and board count giving one of {SPANS_MHZ}')
        a.span = int(round(width))
        a.center = a.band[0] + width * (a.index + 0.5)
    if a.center is None or a.span is None:
        p.error('give --center and --span, or --band/--boards/--index')

    if a.flash:
        flash(a.port, a.build_dir, a.flash_baud)
        time.sleep(1)
    command = f'CYDSET {round(a.center * 1000)} {a.span}' + (f' {a.detector}' if a.detector else '')
    print(command)
    replies = send(a.port, [command, 'CYD?'])
    for r in replies:
        print(r)
    if not replies[0].endswith('OK'):
        sys.exit('board did not accept the slice')


if __name__ == '__main__':
    main()
