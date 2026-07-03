#!/usr/bin/env python3
"""
BSP Backface-Culling Orbit Visualization

Generates N frames orbiting the BSP camera around the scene.
Each frame renders the scene using only triangles that face toward
the BSP camera position (per-triangle backface culling).

Frame 000 is at the convergence point (BSP camera == main camera),
so it should be identical to the non-BSP reference render.
"""

import json
import math
import os
import subprocess
import sys

# ── Configuration ────────────────────────────────────────────────
SCENE_NAME = "bspShowcase"
SCENE_FILE = f"utils/input/{SCENE_NAME}.json"
RENDERER   = "./render"
OUTPUT_DIR = "bsp_rotation_frames"
NUM_STEPS  = 12           # total frames around the full orbit
VERBOSE    = True

# ── Load scene ───────────────────────────────────────────────────
with open(SCENE_FILE) as f:
    scene = json.load(f)

cam = scene["camera"]
lookfrom = cam["lookfrom"]      # [x, y, z]
lookat   = cam["lookat"]        # [x, y, z]

# ── Orbit parameters ─────────────────────────────────────────────
center = lookat                  # orbit around the lookat point
offset = [lookfrom[i] - center[i] for i in range(3)]

# Y offset stays constant (orbit in XZ plane)
y_offset = offset[1]
radius_xz = math.sqrt(offset[0]**2 + offset[2]**2)

# Starting angle (convergence point)
start_angle = math.atan2(offset[2], offset[0])  # atan2(z, x)

# ── Generate frames ──────────────────────────────────────────────
os.makedirs(OUTPUT_DIR, exist_ok=True)

print(f"{'='*60}")
print(f"BSP Backface-Culling Orbit Visualization")
print(f"{'='*60}")
print(f"  Scene:      {SCENE_FILE}")
print(f"  Center:     ({center[0]}, {center[1]}, {center[2]})")
print(f"  Radius XZ:  {radius_xz:.4f}")
print(f"  Y offset:   {y_offset:.4f}")
print(f"  Start angle:{start_angle:.4f} rad ({math.degrees(start_angle):.2f}°)")
print(f"  Frames:     {NUM_STEPS}")
print(f"{'='*60}\n")

for step in range(NUM_STEPS):
    # Angle goes from start_angle full circle around
    angle = start_angle + 2 * math.pi * step / NUM_STEPS

    bx = center[0] + radius_xz * math.cos(angle)
    bz = center[2] + radius_xz * math.sin(angle)
    by = center[1] + y_offset

    output_name = f"bsp_orbit_{step:03d}"
    output_path = os.path.join(OUTPUT_DIR, output_name)

    cmd = [
        RENDERER,
        "-i", SCENE_NAME,
        "--bsp-cam", f"{bx:.6f}", f"{by:.6f}", f"{bz:.6f}",
        "-o", output_path
    ]

    is_convergence = (step == 0)
    label = "CONVERGENCE" if is_convergence else f"step {step:02d}/{NUM_STEPS-1}"

    if VERBOSE:
        print(f"[{label}] BSP cam = ({bx:.4f}, {by:.4f}, {bz:.4f})  "
              f"angle = {math.degrees(angle):6.2f}°")

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"  ERROR: {result.stderr.strip()}")
        continue

    # Extract BSP info line
    for line in result.stdout.split("\n"):
        if "[BSP]" in line:
            print(f"  {line.strip()}")

print(f"\n{'='*60}")
print(f"All frames rendered to {OUTPUT_DIR}/")
print(f"{'='*60}")
print(f"\nTo convert to PNG:  python3 utils/convert_ppm.py {OUTPUT_DIR}/bsp_orbit_000.ppm")
print(f"To create a GIF:    convert {OUTPUT_DIR}/bsp_orbit_*.ppm bsp_orbit.gif")
