#!/usr/bin/env python3
"""Plot an end-to-end mission run: trajectory, trees, depot, apples picked vs left.

Usage: plot_e2e.py trajectory.csv metrics.json out.png [apples_manifest.yaml]
"""
import csv
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402


def main():
    csv_path, metrics_path, png_path = sys.argv[1:4]
    manifest_path = (sys.argv[4] if len(sys.argv) > 4
                     else 'src/applepic_gazebo/worlds/orchard_apples.yaml')
    with open(manifest_path) as f:
        world = yaml.safe_load(f)
    with open(metrics_path) as f:
        m = json.load(f)
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    left = set(m.get('apples_left', []))
    apples = world['apples']

    fig, ax = plt.subplots(figsize=(9, 7))
    if rows:
        ax.plot([float(r['x']) for r in rows], [float(r['y']) for r in rows],
                color='#1f5fa8', lw=1.0, label='robot (/odom)')
    ax.scatter(*zip(*world['trees']), color='#2e7d32', s=90, marker='^', label='trees')
    ax.scatter([world['depot'][0]], [world['depot'][1]], color='#555', s=90, marker='s',
               label='depot')
    gone = [(a['x'], a['y']) for a in apples if a['name'] not in left]
    still = [(a['x'], a['y']) for a in apples if a['name'] in left]
    if gone:
        ax.scatter(*zip(*gone), facecolors='none', edgecolors='#888', s=30,
                   label=f'apples picked ({len(gone)})')
    if still:
        ax.scatter(*zip(*still), color='#d62728', s=30, label=f'apples left ({len(still)})')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'ApplePic e2e: {m["result"]} - unloaded {m["apples_unloaded"]}/'
                 f'{m["apples_total"]}, sim {m["sim_time_s"]:.0f} s, {m["distance_m"]:.0f} m')
    ax.legend(loc='center', frameon=False, fontsize=8)
    fig.savefig(png_path, dpi=100, bbox_inches='tight')
    return 0


if __name__ == '__main__':
    sys.exit(main())
