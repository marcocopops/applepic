"""Pure-Python orchard world generator (no ROS dependency).

Builds an SDF world (gz sim / Harmonic) from a YAML layout: ground, depot zone,
apple trees and red apples scattered on the ground around each tree with a fixed seed.
Also produces a ground-truth apple manifest used later for evaluation.
"""
import math
import random
from dataclasses import dataclass
from xml.sax.saxutils import escape

import yaml


@dataclass(frozen=True)
class Apple:
    name: str
    tree_index: int
    x: float
    y: float


def load_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_apples(config):
    """Return a deterministic list of apples for the given config."""
    rng = random.Random(config['seed'])
    acfg = config['apples']
    n_min, n_max = acfg['per_tree']
    r_min, r_max = acfg['ring']
    spacing = acfg['min_spacing']
    apples = []
    for ti, (tx, ty) in enumerate(config['trees']):
        count = rng.randint(n_min, n_max)
        placed = []
        attempts = 0
        while len(placed) < count:
            attempts += 1
            if attempts > 1000:
                raise ValueError(f'cannot place {count} apples around tree {ti}')
            # sqrt sampling -> uniform density over the annulus
            r = math.sqrt(rng.uniform(r_min ** 2, r_max ** 2))
            theta = rng.uniform(0.0, 2.0 * math.pi)
            x, y = tx + r * math.cos(theta), ty + r * math.sin(theta)
            if all(math.hypot(x - px, y - py) >= spacing for px, py in placed):
                placed.append((x, y))
        for k, (x, y) in enumerate(placed):
            apples.append(Apple(f'apple_t{ti}_{k}', ti, round(x, 4), round(y, 4)))
    return apples


def _material(r, g, b):
    return (f'<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def _ground(center, size):
    cx, cy = center
    sx, sy = size
    return f"""
    <model name="ground">
      <static>true</static>
      <pose>{cx} {cy} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>
          {_material(0.2, 0.5, 0.15)}
        </visual>
      </link>
    </model>"""


def _depot(position, size):
    x, y = position
    sx, sy = size
    # Visual only, slightly above the ground to avoid z-fighting; the robot drives onto it.
    return f"""
    <model name="depot">
      <static>true</static>
      <pose>{x} {y} 0.005 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} 0.01</size></box></geometry>
          {_material(0.1, 0.3, 0.9)}
        </visual>
      </link>
    </model>"""


def _tree(index, position, tcfg):
    x, y = position
    r, h, cr = tcfg['trunk_radius'], tcfg['trunk_height'], tcfg['canopy_radius']
    return f"""
    <model name="tree_{index}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <collision name="trunk_collision">
          <pose>0 0 {h / 2} 0 0 0</pose>
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name="trunk_visual">
          <pose>0 0 {h / 2} 0 0 0</pose>
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          {_material(0.4, 0.25, 0.1)}
        </visual>
        <visual name="canopy_visual">
          <pose>0 0 {h + cr * 0.6} 0 0 0</pose>
          <geometry><sphere><radius>{cr}</radius></sphere></geometry>
          {_material(0.1, 0.35, 0.1)}
        </visual>
      </link>
    </model>"""


def _apple(apple, acfg):
    r, m = acfg['radius'], acfg['mass']
    inertia = 0.4 * m * r * r
    return f"""
    <model name="{escape(apple.name)}">
      <pose>{apple.x} {apple.y} {r} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{m}</mass>
          <inertia><ixx>{inertia:.3e}</ixx><iyy>{inertia:.3e}</iyy><izz>{inertia:.3e}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>{r}</radius></sphere></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>{r}</radius></sphere></geometry>
          {_material(0.85, 0.05, 0.05)}
        </visual>
      </link>
    </model>"""


def build_world_sdf(config, apples=None):
    """Return the full SDF world as a string."""
    if apples is None:
        apples = generate_apples(config)
    sc = config['spherical_coordinates']
    parts = [f"""<?xml version="1.0"?>
<!-- Generated by applepic_gazebo/orchard.py (seed {config['seed']}). Do not edit by hand. -->
<sdf version="1.9">
  <world name="{escape(config['world_name'])}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{sc['latitude_deg']}</latitude_deg>
      <longitude_deg>{sc['longitude_deg']}</longitude_deg>
      <elevation>{sc['elevation']}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 1.0 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>"""]
    ground = config['ground']
    parts.append(_ground(ground.get('center', [0.0, 0.0]), ground['size']))
    parts.append(_depot(config['depot']['position'], config['depot']['size']))
    for i, pos in enumerate(config['trees']):
        parts.append(_tree(i, pos, config['tree']))
    for apple in apples:
        parts.append(_apple(apple, config['apples']))
    parts.append('\n  </world>\n</sdf>\n')
    return ''.join(parts)


def build_manifest(config, apples):
    """Ground-truth description of the generated world (for metrics and tests)."""
    return {
        'world_name': config['world_name'],
        'seed': config['seed'],
        'depot': list(config['depot']['position']),
        'trees': [list(t) for t in config['trees']],
        'apples': [{'name': a.name, 'tree': a.tree_index, 'x': a.x, 'y': a.y} for a in apples],
    }
