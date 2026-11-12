#!/usr/bin/env python3
"""Record README visuals headless: run the mission with extra "photo" cameras and save their frames.

Three cameras are added to a copy of the world (they are not part of the robot):
  overview  static, oblique view of the whole orchard
  depot     static, low view of the depot (start and unloading)
  chase     follows the robot from outside the nearest tree, so the canopy never hides it
            (moved with the gz set_pose service)
Frames go to OUT/<camera>/<sim_ms>.jpg, robot poses and /mission/stats to OUT/events.jsonl.
`scripts/make_media.py` then turns them into the GIF and stills of docs/media/.

Usage: scripts/record_media.py [--out artifacts/media_raw] [--rtf 2] [--sim-limit 900]
Needs a built workspace (make build) and no other simulation running.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from PIL import Image as PILImage
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(ROOT, 'src/applepic_gazebo/worlds/orchard.sdf')
ROBOT = 'applepic'
TREES = [(4.0, 4.0), (10.0, 4.0), (16.0, 4.0), (16.0, 10.0), (10.0, 10.0), (4.0, 10.0)]  # orchard.yaml
CHASE_BACK, CHASE_HEIGHT = 2.6, 3.0   # chase camera: metres behind the robot (away from the tree), height


def look_at(eye, target):
    """Roll-pitch-yaw of a gz camera (looking along +X) at `eye` aimed at `target`."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def camera_sdf(name, eye, target, width, height, hfov, rate, static=True):
    r, p, y = look_at(eye, target)
    return f"""
    <model name="{name}_camera">
      <static>{'true' if static else 'false'}</static>
      <pose>{eye[0]} {eye[1]} {eye[2]} {r} {p} {y}</pose>
      <link name="link">
        <gravity>false</gravity>
        <sensor name="{name}" type="camera">
          <topic>media/{name}/image</topic>
          <update_rate>{rate}</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>120</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""


def build_world(path):
    cameras = (
        camera_sdf('overview', (-4.0, -6.0, 12.0), (8.5, 6.0, 0.0), 1600, 900, 1.1, 0.5)
        + camera_sdf('depot', (2.6, -2.2, 1.5), (0.0, 0.3, 0.1), 1280, 720, 1.0, 0.5)
        + camera_sdf('chase', (-CHASE_BACK, -CHASE_BACK, CHASE_HEIGHT), (0.0, 0.0, 0.2),
                     1280, 720, 1.1, 2.0))
    sdf = open(WORLD, encoding='utf-8').read().replace('</world>', cameras + '\n  </world>', 1)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(sdf)


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def gz_call(service, reqtype, req, timeout_ms=1000):
    # A subprocess, not GzNode.request: the Python binding holds the GIL while waiting.
    out = subprocess.run(['gz', 'service', '-s', service, '--reqtype', reqtype,
                          '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms),
                          '--req', req], capture_output=True, text=True)
    return 'true' in out.stdout


class Recorder:
    def __init__(self, out):
        self.out = out
        self.lock = threading.Lock()
        self.robot = None          # (sim_t, x, y, yaw)
        self.sim_t = 0.0
        self.stats = {}
        self.counts = {}
        self.events = open(os.path.join(out, 'events.jsonl'), 'w', encoding='utf-8')

    def log(self, **kw):
        with self.lock:
            self.events.write(json.dumps(kw) + '\n')

    def on_poses(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == ROBOT:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    first = self.robot is None or t - self.robot[0] >= 0.5
                    self.robot = (t, p.position.x, p.position.y, yaw)
                    self.sim_t = t
                if first:
                    self.log(kind='pose', t=round(t, 2), x=round(p.position.x, 3),
                             y=round(p.position.y, 3), yaw=round(yaw, 3))
                return

    def on_image(self, name):
        def cb(msg):
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            os.makedirs(os.path.join(self.out, name), exist_ok=True)
            PILImage.fromarray(img).save(
                os.path.join(self.out, name, f'{int(round(t * 1000)):08d}.jpg'), quality=90)
            with self.lock:
                self.counts[name] = self.counts.get(name, 0) + 1
        return cb

    def on_stats(self, msg):
        stats = json.loads(msg.data)
        with self.lock:
            self.stats = stats
            t = self.sim_t
        self.log(kind='stats', t=round(t, 2), **stats)


def chase_goal(x, y):
    """Camera eye and target for a robot at (x, y): on the line tree -> robot, beyond the robot."""
    tx, ty = min(TREES, key=lambda t: math.hypot(x - t[0], y - t[1]))
    d = math.hypot(x - tx, y - ty)
    ux, uy = ((x - tx) / d, (y - ty) / d) if 0.3 < d < 4.0 else (-0.7071, -0.7071)
    eye = (x + CHASE_BACK * ux, y + CHASE_BACK * uy, CHASE_HEIGHT)
    target = (x - 0.6 * ux, y - 0.6 * uy, 0.2)
    return eye + target


def follow(rec, stop):
    """Move the chase camera towards chase_goal, low-pass filtered so it orbits smoothly."""
    cur = None
    while not stop.is_set():
        with rec.lock:
            robot = rec.robot
        if robot is not None:
            goal = chase_goal(robot[1], robot[2])
            cur = goal if cur is None else tuple(c + 0.2 * (g - c) for c, g in zip(cur, goal))
            r, p, yw = look_at(cur[:3], cur[3:])
            qx, qy, qz, qw = quat(r, p, yw)
            gz_call('/world/orchard/set_pose', 'gz.msgs.Pose',
                    f'name: "chase_camera", position: {{x: {cur[0]:.3f}, y: {cur[1]:.3f}, '
                    f'z: {cur[2]:.3f}}}, orientation: {{x: {qx:.5f}, y: {qy:.5f}, '
                    f'z: {qz:.5f}, w: {qw:.5f}}}', 300)
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(ROOT, 'artifacts/media_raw'))
    ap.add_argument('--rtf', type=float, default=2.0)
    ap.add_argument('--sim-limit', type=float, default=900.0)
    ap.add_argument('--wall-limit', type=float, default=1800.0)
    args = ap.parse_args()
    args.out = os.path.abspath(args.out)
    os.makedirs(args.out, exist_ok=True)
    # SIGTERM (e.g. from `timeout`) must still stop the simulation in the finally block.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
    world = os.path.join(args.out, 'orchard_media.sdf')
    build_world(world)

    launch = subprocess.Popen(
        ['ros2', 'launch', 'applepic_bringup', 'mission.launch.py', 'headless:=true',
         f'world:={world}', 'loops:=1', 'capacity:=10'],
        stdout=open(os.path.join(args.out, 'launch.log'), 'w'), stderr=subprocess.STDOUT,
        start_new_session=True)   # own process group, stopped by PGID

    rec = Recorder(args.out)
    gz = GzNode()
    gz.subscribe(Pose_V, '/world/orchard/dynamic_pose/info', rec.on_poses)
    for name in ('overview', 'depot', 'chase'):
        gz.subscribe(Image, f'/media/{name}/image', rec.on_image(name))
    rclpy.init()
    ros = rclpy.create_node('media_recorder')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ros.create_subscription(String, '/mission/stats', rec.on_stats, latched)

    stop = threading.Event()
    threading.Thread(target=follow, args=(rec, stop), daemon=True).start()
    status = 1
    try:
        if args.rtf != 1.0:
            for _ in range(60):
                if gz_call('/world/orchard/set_physics', 'gz.msgs.Physics',
                           f'max_step_size: 0.001, real_time_factor: {args.rtf}', 2000):
                    print(f'real-time factor set to {args.rtf}', flush=True)
                    break
                time.sleep(1)
        start, last = time.time(), 0.0
        while time.time() - start < args.wall_limit:
            rclpy.spin_once(ros, timeout_sec=0.5)
            with rec.lock:
                state, t, counts = rec.stats.get('state'), rec.sim_t, dict(rec.counts)
            if time.time() - last > 30:
                last = time.time()
                print(f'wall {time.time() - start:5.0f} s  sim {t:6.1f} s  state {state}  '
                      f'unloaded {rec.stats.get("unloaded")}  frames {counts}', flush=True)
            if state == 'DONE':
                print('mission DONE', flush=True)
                status = 0
                break
            if t > args.sim_limit:
                print('sim limit reached', flush=True)
                break
            if launch.poll() is not None:
                print('the launch stopped, see launch.log', flush=True)
                break
    finally:
        stop.set()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        rec.events.close()
        ros.destroy_node()
        rclpy.shutdown()
    print(json.dumps(rec.stats))
    return status


if __name__ == '__main__':
    sys.exit(main())
