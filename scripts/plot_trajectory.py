#!/usr/bin/env python3
"""Plot a waypoint_follower trajectory CSV over the orchard layout and print metrics.

Usage: plot_trajectory.py trajectory.csv out.png [orchard.yaml]
"""
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402


def main():
    csv_path, png_path = sys.argv[1], sys.argv[2]
    cfg_path = sys.argv[3] if len(sys.argv) > 3 else 'src/applepic_gazebo/config/orchard.yaml'
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    xs = [float(r['x']) for r in rows]
    ys = [float(r['y']) for r in rows]
    t = [float(r['t']) for r in rows]
    trees = [tuple(p) for p in cfg['trees']]
    dist = sum(math.dist(a, b) for a, b in zip(zip(xs, ys), list(zip(xs, ys))[1:]))
    min_clear = min(math.dist((x, y), tr) for x, y in zip(xs, ys) for tr in trees)
    print(f'samples={len(rows)} sim_time={t[-1] - t[0]:.1f}s distance={dist:.1f}m '
          f'min_trunk_distance={min_clear:.2f}m final=({xs[-1]:.2f}, {ys[-1]:.2f})')

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, color='#1f5fa8', lw=1.5, label='robot (/odom)')
    ax.scatter(*zip(*trees), color='#2e7d32', s=80, marker='^', label='trees')
    ax.scatter([cfg['depot']['position'][0]], [cfg['depot']['position'][1]],
               color='#555', s=80, marker='s', label='depot')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(sys.argv[4] if len(sys.argv) > 4 else 'ApplePic waypoint loop')
    ax.legend(loc='lower right', frameon=False)
    fig.savefig(png_path, dpi=100, bbox_inches='tight')
    return 0 if min_clear > 1.0 else 1


if __name__ == '__main__':
    sys.exit(main())
