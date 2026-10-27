#!/usr/bin/env python3
"""Flash a batch of ESP32 boards one after another, hands-free.

Plug boards in one at a time. Each board the watcher has not seen (by MAC)
is flashed from an ESP-IDF build directory (its flash_args), optionally
checked over serial, and announced with a sound: Glass when it is done and
ready to unplug, Basso on failure. Known boards are skipped, so re-plugging
one is harmless. Only boards whose chip type matches --chip are touched.

Examples (activate the ESP-IDF environment first, for esptool and pyserial):

  # Cheap Yellow Displays on a CH340: check the display came up, keep a board list
  python tools/flash_watch.py --build-dir build-cyd --chip ESP32-D0WD --check 'CYD?' --expect 'CYD PRESENT' \
      --record flashed-boards.tsv

  # Any ESP32-S3 on native USB, no serial check, skip two boards already done
  python tools/flash_watch.py --build-dir build-s3 --chip ESP32-S3 \
      --ports '/dev/cu.usbmodem*' --known aa:bb:cc:00:00:01 aa:bb:cc:00:00:02
"""
import argparse
import glob
import hashlib
import re
import subprocess
import sys
import time
from pathlib import Path


def log(*args):
    print(time.strftime('%H:%M:%S'), *args, flush=True)


def chime(name, enabled):
    """Audible cue that also works from a background process (no terminal bell)."""
    sound = Path(f'/System/Library/Sounds/{name}.aiff')
    if enabled and sound.exists():
        subprocess.Popen(['afplay', str(sound)])
    elif enabled:
        print('\a', end='', flush=True)


def image_fingerprint(build_dir):
    """Short hash of every image flash_args writes: tells builds apart."""
    h = hashlib.sha256()
    for token in (build_dir / 'flash_args').read_text().split():
        path = build_dir / token
        if not token.startswith('-') and path.is_file():
            h.update(path.read_bytes())
    return h.hexdigest()[:12]


def load_record(path):
    """MACs from the first field of each line of a --record file (# comments)."""
    if not path or not path.exists():
        return set()
    macs = set()
    for line in path.read_text().splitlines():
        if line and not line.startswith('#'):
            macs.add(line.split()[0].lower())
    return macs


def append_record(path, mac, kind, build_dir, image, status):
    new = not path.exists()
    with path.open('a') as f:
        if new:
            f.write('# mac\tchip\tflashed\tbuild\timage\tstatus\n')
        f.write(f'{mac}\t{kind}\t{time.strftime("%Y-%m-%d %H:%M:%S")}\t{build_dir}\t{image}\t{status}\n')


def identify(port):
    out = subprocess.run([sys.executable, '-m', 'esptool', '-p', port, 'chip-id'], capture_output=True, text=True,
                         timeout=60).stdout
    kind = re.search(r'Chip type:\s+(\S+)', out)
    mac = re.search(r'MAC:\s+([0-9a-f:]{17})', out)
    return (kind.group(1) if kind else None), (mac.group(1) if mac else None)


def check(port, command, baud, boot_wait):
    """Send one command and return the printable reply (boot noise removed)."""
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, baud, 1, False, False
    s.open()
    try:
        time.sleep(boot_wait)  # opening a CH340 port resets many boards
        s.reset_input_buffer()
        s.write((command + '\n').encode())
        reply = ''.join(c for c in s.readline().decode(errors='replace') if c.isprintable()).strip()
        return reply.split('  ')[-1].strip()
    finally:
        s.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--build-dir', type=Path, required=True, help='ESP-IDF build directory containing flash_args')
    p.add_argument('--chip', default='ESP32', help='only flash boards whose chip type starts with this')
    p.add_argument('--ports', nargs='+', default=['/dev/cu.usbserial-*', '/dev/cu.wchusbserial*'],
                   help='serial port globs to watch')
    p.add_argument('--baud', type=int, default=460800, help='flashing baud rate')
    p.add_argument('--known', nargs='*', default=[], help='MACs to skip (already programmed)')
    p.add_argument('--record', type=Path, help='board list (TSV): its MACs are skipped, new boards are appended')
    p.add_argument('--check', help='serial command sent after flashing, e.g. "CYD?"')
    p.add_argument('--expect', help='reply prefix that means the firmware came up')
    p.add_argument('--check-baud', type=int, default=2000000)
    p.add_argument('--boot-wait', type=float, default=4.0, help='seconds between opening the port and --check')
    p.add_argument('--minutes', type=float, default=60, help='stop watching after this long')
    p.add_argument('--quiet', action='store_true', help='no sounds')
    a = p.parse_args()

    flash_args = a.build_dir / 'flash_args'
    if not flash_args.exists():
        p.error(f'{flash_args} not found: build the project first')
    seen = {m.lower() for m in a.known} | load_record(a.record)
    image = image_fingerprint(a.build_dir)
    done = []
    end = time.time() + a.minutes * 60
    log(f'watching {" ".join(a.ports)} for {a.chip} boards; flashing {a.build_dir} (image {image}); '
        f'{len(seen)} known')

    while time.time() < end:
        ports = sorted({q for g in a.ports for q in glob.glob(g)})
        if not ports:
            time.sleep(1)
            continue
        port = ports[0]
        try:
            kind, mac = identify(port)
        except Exception as e:
            log('probe failed on', port, e)
            time.sleep(2)
            continue
        if not kind or not mac or not kind.startswith(a.chip):
            log('ignoring', port, kind, mac)
        elif mac in seen:
            log('known board', mac, 'on', port, '- unplug it and plug in the next one')
        else:
            log('new board', mac, '(' + kind + ') on', port, '- flashing')
            r = subprocess.run([sys.executable, '-m', 'esptool', '-p', port, '-b', str(a.baud), '--before',
                                'default-reset', '--after', 'hard-reset', 'write-flash', '@flash_args'],
                               cwd=a.build_dir, capture_output=True, text=True)
            if r.returncode != 0:
                log('FLASH FAILED', mac, (r.stdout + r.stderr)[-300:].replace('\n', ' | '))
                chime('Basso', not a.quiet)
                time.sleep(3)
                continue
            status = 'flashed'
            if a.check:
                time.sleep(1)
                try:
                    reply = check(port, a.check, a.check_baud, a.boot_wait)
                except Exception as e:
                    reply = f'check failed: {e}'
                ok = not a.expect or reply.startswith(a.expect)
                status = ('ok: ' if ok else 'CHECK FAILED: ') + reply[:160]
            else:
                ok = True
            log(mac, status)
            if ok:
                seen.add(mac)
                done.append(mac)
                if a.record:
                    append_record(a.record, mac, kind, a.build_dir, image, status)
            chime('Glass' if ok else 'Basso', not a.quiet)
        while glob.glob(port) and time.time() < end:  # wait for unplug
            time.sleep(0.5)
        if not glob.glob(port):
            log('unplugged', mac)

    log(f'finished; programmed {len(done)} board(s):', ', '.join(done) or 'none')


if __name__ == '__main__':
    main()
