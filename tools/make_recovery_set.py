#!/usr/bin/env python3
"""Assemble a USB flash set: current build in ota_0 + published firmware in factory.

usage: make_recovery_set.py <build_dir> <out_dir> [track]   (track: dashkit | dashkit-juniper)

NVS (0x9000: bonds, Tesla key, settings) is deliberately not written.
"""
import hashlib, json, shutil, struct, sys, urllib.request, zlib
from pathlib import Path

TOKEN = '68bcc03a-3951-454d-8f35-f5ab06c0ed0a'  # must match tools/publish_ota.sh
BASE = 'https://firebasestorage.googleapis.com/v0/b/dashkit-connect.firebasestorage.app/o/'
build, out = Path(sys.argv[1]), Path(sys.argv[2])
track = sys.argv[3] if len(sys.argv) > 3 else 'dashkit'

def get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return r.read()

m = json.loads(get(f'{BASE}{track}%2Fmanifest.json?alt=media&token={TOKEN}'))
if not m['url'].startswith('https://'):
    sys.exit('manifest url is not https')
fw = get(m['url'])
if m.get('sha256') and hashlib.sha256(fw).hexdigest() != m['sha256'].lower():
    sys.exit('published firmware sha256 mismatch')
if len(fw) > 0x200000:
    sys.exit('published firmware larger than the factory slot')

out.mkdir(parents=True, exist_ok=True)
(out / f'published-{m["version"]}.bin').write_bytes(fw)
shutil.copy(build / 'partition_table' / 'partition-table.bin', out)
shutil.copy(build / 'dashkit-firmware.bin', out)

# otadata selecting ota_0 (seq 1): entry = seq, 20B label, state, crc(seq). Same as otatool.py.
entry = bytearray(b'\xff' * 32)
entry[0:4] = struct.pack('<I', 1)
entry[28:32] = struct.pack('<I', zlib.crc32(struct.pack('<I', 1), 0xFFFFFFFF))
(out / 'otadata-ota0.bin').write_bytes(bytes(entry) + b'\xff' * (0x2000 - 32))

(out / 'FLASH.txt').write_text(
    'esptool.py --chip esp32s3 write_flash \\\n'
    '  0x8000 partition-table.bin 0xf000 otadata-ota0.bin \\\n'
    f'  0x20000 dashkit-firmware.bin 0x420000 published-{m["version"]}.bin\n')
print(f'recovery set: published {m["version"]} ({len(fw)} B) in {out}')
