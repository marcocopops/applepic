#!/usr/bin/env python3
"""Headless check of a world: add an overhead camera, run gz sim, save one PNG.

Usage: scripts/capture_overhead.py WORLD.sdf OUT.png [--x X --y Y --z Z]
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PILImage

CAMERA = """
    <model name="overhead_camera">
      <static>true</static>
      <pose>{x} {y} {z} 0 1.5707 0</pose>
      <link name="link">
        <sensor name="overhead" type="camera">
          <topic>overhead/image</topic>
          <update_rate>2</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>1.4</horizontal_fov>
            <image><width>1024</width><height>768</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
  </world>"""


def main():
    p = argparse.ArgumentParser()
    p.add_argument('world')
    p.add_argument('out')
    p.add_argument('--x', type=float, default=10.0)
    p.add_argument('--y', type=float, default=7.0)
    p.add_argument('--z', type=float, default=14.0)
    p.add_argument('--timeout', type=float, default=40.0)
    args = p.parse_args()

    sdf = open(args.world, encoding='utf-8').read()
    sdf = sdf.replace('</world>', CAMERA.format(x=args.x, y=args.y, z=args.z), 1)
    tmp = tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False)
    tmp.write(sdf)
    tmp.close()

    frames = []
    node = Node()
    node.subscribe(Image, '/overhead/image', frames.append)
    sim = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', tmp.name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.time() + args.timeout
        while not frames and time.time() < deadline:
            time.sleep(0.2)
    finally:
        sim.terminate()
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            sim.kill()
        os.unlink(tmp.name)
    if not frames:
        print('no image received', file=sys.stderr)
        return 1
    msg = frames[-1]
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    PILImage.fromarray(img).save(args.out)
    print(f'saved {args.out} ({msg.width}x{msg.height})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
