#!/usr/bin/env python3
"""Debug PNG files from the orbit render."""

import struct
import zlib
import sys

def analyze_png(path):
    with open(path, 'rb') as f:
        data = f.read()
    print(f'{path}:')
    print(f'  File size: {len(data)} bytes')

    pos = 8
    while pos < len(data):
        length = struct.unpack('>I', data[pos:pos+4])[0]
        ctype = data[pos+4:pos+8].decode('ascii', errors='replace')
        chunk_data = data[pos+8:pos+8+length]
        if ctype == 'IHDR':
            w, h = struct.unpack('>II', chunk_data[:8])
            print(f'  IHDR: {w}x{h}, bit_depth={chunk_data[8]}, color_type={chunk_data[9]}')
        elif ctype == 'IDAT':
            dec = zlib.decompress(chunk_data)
            print(f'  IDAT: {length} compressed -> {len(dec)} decompressed')
            # Show first 15 RGB pixels
            first_pixels = []
            for i in range(0, min(45, len(dec)), 3):
                first_pixels.append(tuple(dec[i:i+3]))
            print(f'  First 15 pixels: {first_pixels}')
            # Expected: 800*600*3 = 1,440,000 bytes
            expected = w * h * 3
            print(f'  Expected decompressed: {expected}, got: {len(dec)}')
            if len(dec) != expected:
                print(f'  ** WRONG SIZE: should be {expected} but is {len(dec)}')
        pos += 12 + length

def check_ppm(path):
    with open(path, 'r') as f:
        header = f.readline().strip()
        assert header == 'P3', f'Not P3: {header}'
        line = f.readline()
        while line.startswith('#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        # Count remaining non-empty tokens
        remaining = f.read().split()
        n_pixels = len(remaining) // 3
        print(f'  PPM: {w}x{h}, maxval={maxval}, {len(remaining)} values = {n_pixels} pixels')
        print(f'  Expected pixels: {w*h}, actual: {n_pixels}')
        # Show first few pixel values
        first_vals = [int(remaining[i]) for i in range(min(9, len(remaining)))]
        print(f'  First 9 values (3 pixels): {first_vals}')
        return w, h, remaining

if __name__ == '__main__':
    base = 'bsp_rotation_frames/bsp_orbit_000'
    print('=== PPM Analysis ===')
    check_ppm(f'{base}.ppm')
    print()
    print('=== PNG Analysis ===')
    analyze_png(f'{base}.png')
