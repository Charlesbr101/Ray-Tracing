#!/usr/bin/env python3
"""Convert PPM P3 (text) files to PNG. Pure Python, no PIL needed."""

import glob
import struct
import sys
import zlib


def chunk(ctype, data):
    c = ctype + data
    return (struct.pack('>I', len(data)) + c +
            struct.pack('>I', zlib.crc32(c) & 0xffffffff))


def ppm_to_png(ppm_path, png_path=None):
    with open(ppm_path, 'r') as f:
        header = f.readline().strip()
        if header != 'P3':
            print(f'Unsupported format: {header}')
            return False
        line = f.readline()
        while line.startswith('#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        pixels = []
        for line in f:
            for v in line.split():
                pixels.append(int(v))

    if png_path is None:
        png_path = ppm_path.rsplit('.', 1)[0] + '.png'

    # Build raw pixel rows with PNG filter byte (0=None) before each row.
    # Use bytearray for fast construction.
    raw = bytearray(h * (1 + w * 3))
    off = 0
    for row in range(h):
        raw[off] = 0  # filter type: None
        off += 1
        for col in range(w):
            i = (row * w + col) * 3
            raw[off] = pixels[i]
            raw[off + 1] = pixels[i + 1]
            raw[off + 2] = pixels[i + 2]
            off += 3
    raw = bytes(raw)

    sig = b'\x89PNG\r\n\x1a\n'
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    idat = zlib.compress(raw)
    iend = b''

    with open(png_path, 'wb') as f:
        f.write(sig)
        f.write(chunk(b'IHDR', ihdr))
        f.write(chunk(b'IDAT', idat))
        f.write(chunk(b'IEND', iend))
    print(f'Written {png_path}')
    return True


if __name__ == '__main__':
    if len(sys.argv) > 1:
        for pattern in sys.argv[1:]:
            for path in glob.glob(pattern):
                ppm_to_png(path)
    else:
        print('Usage: python3 ppm_to_png.py <ppm_file_or_glob>')
