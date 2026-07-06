#!/usr/bin/env python3
"""
Orbit Visualization — Main Camera or BSP Camera.

ORBIT_TYPE controls which camera orbits:

  "main" — The MAIN camera moves around the scene. The BSP camera stays
           fixed at the original camera position, and only triangles that
           face toward that fixed point are rendered. The viewpoint changes
           while the visible triangle set stays constant.

  "bsp"  — The BSP camera moves around the scene. The MAIN camera stays
           fixed at its original position. The viewpoint is constant, but
           which triangles are front-facing changes — producing a shifting
           visibility effect.

Frame 000 is always at convergence (both cameras aligned), giving a
pixel-identical result to a normal non-BSP render.
"""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

# ═══════════════════════════════════════════════════════════════════
#  Configuration — tweak these variables
# ═══════════════════════════════════════════════════════════════════

SCENE_NAME = "bspShowcase"       # scene name (loaded from utils/input/<name>.json)
RENDERER   = "./render"          # path to the compiled renderer
OUTPUT_DIR = "bsp_rotation_frames"  # where PPM / PNG files are written
NUM_STEPS  = 24                  # frames around the full orbit
VERBOSE    = True                # print per-frame info

# Orbit type: "main" = main camera orbits, "bsp" = BSP camera orbits
ORBIT_TYPE = "main"

# PNG conversion
AUTO_CONVERT = True              # automatically convert each PPM → PNG
DELETE_PPM   = False             # delete the PPM after PNG conversion?

# ═══════════════════════════════════════════════════════════════════

# ── Helpers ───────────────────────────────────────────────────────

def ppm_to_png(ppm_path):
    """Convert a single PPM to PNG using the external converter script."""
    result = subprocess.run(
        [sys.executable, "utils/ppm_to_png.py", ppm_path],
        capture_output=True, text=True
    )
    for line in result.stdout.splitlines():
        if "Written" in line:
            if VERBOSE:
                print(f"  {line.strip()}")
    return result.returncode == 0


def orbit_positions(center, offset, num_steps):
    """Yield (x, y, z) positions around an XZ orbit."""
    y_off = offset[1]
    r_xz = math.sqrt(offset[0]**2 + offset[2]**2)
    start_angle = math.atan2(offset[2], offset[0])
    for step in range(num_steps):
        angle = start_angle + 2 * math.pi * step / num_steps
        x = center[0] + r_xz * math.cos(angle)
        z = center[2] + r_xz * math.sin(angle)
        y = center[1] + y_off
        yield x, y, z, angle


# ── Load scene ───────────────────────────────────────────────────

with open(f"utils/input/{SCENE_NAME}.json") as f:
    scene = json.load(f)

cam = scene["camera"]
lookfrom = cam["lookfrom"]     # [x, y, z]
lookat   = cam["lookat"]       # [x, y, z]
center = lookat
offset = [lookfrom[i] - center[i] for i in range(3)]

# Build the fixed BSP camera definition (stays at original camera position)
bsp_fixed = {
    "lookfrom": lookfrom[:],
    "lookat":   lookat[:],
    "upVector": cam.get("upVector", [0, 1, 0]),
    "image_width":  cam.get("image_width", 800),
    "image_height": cam.get("image_height", 600),
    "screen_distance": cam.get("screen_distance", 0.75),
}

# ── Print summary ────────────────────────────────────────────────

os.makedirs(OUTPUT_DIR, exist_ok=True)

mode_label = {
    "main": "Main camera orbits — BSP camera FIXED",
    "bsp":  "BSP camera orbits — Main camera FIXED",
}

print(f"{'='*70}")
print(f"Orbit Visualization — {mode_label.get(ORBIT_TYPE, ORBIT_TYPE)}")
print(f"{'='*70}")
print(f"  Scene:        {SCENE_NAME}")
print(f"  Camera:       ({lookfrom[0]}, {lookfrom[1]}, {lookfrom[2]})")
print(f"  Lookat:       ({lookat[0]}, {lookat[1]}, {lookat[2]})")
print(f"  Orbit radius: {math.sqrt(offset[0]**2 + offset[2]**2):.4f} (XZ)")
print(f"  Y offset:     {offset[1]:.4f}")
print(f"  Frames:       {NUM_STEPS}")
print(f"  PNG convert:  {'ON' if AUTO_CONVERT else 'OFF'}"
      f"{' (delete PPM)' if DELETE_PPM else ''}")
print(f"{'='*70}\n")

# ── Generate frames ──────────────────────────────────────────────

tmp_dir = tempfile.mkdtemp(prefix="bsp_orbit_")

for step, (ox, oy, oz, angle) in enumerate(
        orbit_positions(center, offset, NUM_STEPS)):

    output_name = f"bsp_orbit_{step:03d}"
    output_path = os.path.join(OUTPUT_DIR, output_name)
    is_convergence = (step == 0)
    label = "CONVERGENCE" if is_convergence else f"step {step:02d}/{NUM_STEPS-1}"

    # Build a modified scene JSON with the appropriate main camera
    # and customBSPCamera fields.
    mod_scene = scene.copy()

    if ORBIT_TYPE == "main":
        # ── Main camera orbits, BSP camera fixed ──────────────
        mod_scene["camera"] = {
            "lookfrom": [ox, oy, oz],
            "lookat":   lookat[:],
            "upVector": cam.get("upVector", [0, 1, 0]),
            "image_width":  cam.get("image_width", 800),
            "image_height": cam.get("image_height", 600),
            "screen_distance": cam.get("screen_distance", 0.75),
        }
        mod_scene["customBSPCamera"] = bsp_fixed

        if VERBOSE:
            print(f"[{label}] Main cam = ({ox:.4f}, {oy:.4f}, {oz:.4f})  "
                  f"angle = {math.degrees(angle):6.2f}°")

    else:
        # ── BSP camera orbits, main camera fixed ──────────────
        mod_scene["customBSPCamera"] = {
            "lookfrom": [ox, oy, oz],
            "lookat":   lookat[:],
            "upVector": cam.get("upVector", [0, 1, 0]),
            "image_width":  cam.get("image_width", 800),
            "image_height": cam.get("image_height", 600),
            "screen_distance": cam.get("screen_distance", 0.75),
        }

        if VERBOSE:
            print(f"[{label}] BSP cam = ({ox:.4f}, {oy:.4f}, {oz:.4f})  "
                  f"angle = {math.degrees(angle):6.2f}°")

    # Write the modified scene JSON
    tmp_json = os.path.join(tmp_dir, f"frame_{step:03d}.json")
    with open(tmp_json, "w") as f:
        json.dump(mod_scene, f, indent=4)

    # Render using the temp JSON (no --bsp-cam flag — it's in the JSON now)
    cmd = [
        RENDERER, "-f", tmp_json,
        "-o", output_path,
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"  ERROR: {result.stderr.strip()}")
        for line in result.stdout.split("\n"):
            print(f"  | {line}")
        continue

    # Print BSP / timing info
    for line in result.stdout.split("\n"):
        if "[BSP]" in line or "Execution time" in line:
            print(f"  {line.strip()}")

    # ── Convert to PNG ─────────────────────────────────────────
    if AUTO_CONVERT:
        ppm_path = output_path + ".ppm"
        if os.path.exists(ppm_path):
            ppm_to_png(ppm_path)
            if DELETE_PPM:
                os.remove(ppm_path)

# Cleanup
shutil.rmtree(tmp_dir, ignore_errors=True)

print(f"\n{'='*70}")
print(f"Done — {NUM_STEPS} frames in {OUTPUT_DIR}/")
print(f"{'='*70}")
