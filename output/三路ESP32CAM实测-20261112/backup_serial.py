"""Read each board with one in-flight block; verify the stub's MD5 before saving."""
import hashlib
import json
from pathlib import Path
import struct
import esptool

out = Path(__file__).resolve().parent
results = []
for port in ['COM3', 'COM5', 'COM11']:
    esp = esptool.detect_chip(port)
    try:
        esp = esptool.run_stub(esp)
        esp.change_baud(460800)
        esptool.attach_flash(esp)
        length = 0x400000
        esp.check_command('read flash', esp.ESP_CMDS['READ_FLASH'],
                          struct.pack('<IIII', 0, length, 4096, 1))
        data = bytearray()
        while len(data) < length:
            esp._port.timeout = 5
            packet = esp.read()
            if len(packet) != 4096:
                raise RuntimeError(f'{port}: corrupt block at {len(data)}: {len(packet)} bytes')
            data.extend(packet)
            esp.write(struct.pack('<I', len(data)))
            if len(data) % 0x80000 == 0:
                print(f'{port}: {len(data)}/{length}', flush=True)
        digest = esp.read()
        if digest != hashlib.md5(data).digest():
            raise RuntimeError(f'{port}: backup MD5 mismatch')
        path = out / (port + '-before-full-4MB.bin')
        path.write_bytes(data)
        results.append(dict(port=port, size=len(data), sha256=hashlib.sha256(data).hexdigest(), md5_verified=True))
        (out / 'backup-manifest.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
        print(f'{port}: backup verified', flush=True)
    finally:
        esp.hard_reset()
        esp._port.close()
