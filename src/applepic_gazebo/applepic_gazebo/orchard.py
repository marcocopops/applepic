"""Pure-Python orchard world generator (no ROS dependency).

Builds an SDF world (gz sim / Harmonic) from a YAML layout: ground, depot zone,
apple trees and red apples scattered on the ground around each tree with a fixed seed.
Also produces a ground-truth apple manifest used later for evaluation.
"""
import math
import random
from dataclasses import dataclass, replace
from xml.sax.saxutils import escape

import yaml


@dataclass(frozen=True)
class Apple:
    name: str
    tree_index: int
    x: float
    y: float
    variant: str = 'ripe'   # key of apples.variants in the config
    hidden: bool = False    # a grass tuft was planted right next to it


DEFAULT_VARIANT = {'color': [0.85, 0.05, 0.05]}


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
    return _dress_apples(config, apples)


def _dress_apples(config, apples):
    """Draw the look of each apple. Own generator: positions do not depend on these options."""
    variants = config['apples'].get('variants')
    hidden_ratio = config.get('grass', {}).get('hidden_ratio', 0.0)
    if not variants and not hidden_ratio:
        return apples
    rng = random.Random(f'{config["seed"]}-look')
    names = list(variants or ['ripe'])
    weights = [variants[n].get('weight', 1.0) for n in names] if variants else [1.0]
    return [replace(a, variant=rng.choices(names, weights)[0],
                    hidden=rng.random() < hidden_ratio) for a in apples]


def generate_grass(config, apples):
    """Return [(tree_index, [(x, y, height, yaw, tilt), ...]), ...]: blades in world frame.

    Tufts next to the `hidden` apples, plus `tufts_per_tree` scattered in the apple ring. Each
    tuft is kept clear of the trunk foot, which the localization needs to see.
    """
    gcfg = config.get('grass')
    if not gcfg:
        return []
    rng = random.Random(f'{config["seed"]}-grass')
    r_min, r_max = config['apples']['ring']
    trunk_clearance = gcfg.get('trunk_clearance', 0.3)
    h_min, h_max = gcfg['height']
    out = []
    for ti, (tx, ty) in enumerate(config['trees']):
        centers = []
        for a in apples:
            if a.tree_index == ti and a.hidden:
                angle = rng.uniform(0.0, 2.0 * math.pi)
                centers.append((a.x + gcfg['hidden_offset'] * math.cos(angle),
                                a.y + gcfg['hidden_offset'] * math.sin(angle)))
        for _ in range(gcfg['tufts_per_tree']):
            r = math.sqrt(rng.uniform(r_min ** 2, r_max ** 2))
            theta = rng.uniform(0.0, 2.0 * math.pi)
            centers.append((tx + r * math.cos(theta), ty + r * math.sin(theta)))
        blades = []
        for cx, cy in centers:
            for _ in range(gcfg['blades_per_tuft']):
                r = gcfg['tuft_radius'] * math.sqrt(rng.random())
                theta = rng.uniform(0.0, 2.0 * math.pi)
                x, y = cx + r * math.cos(theta), cy + r * math.sin(theta)
                if math.hypot(x - tx, y - ty) < trunk_clearance:
                    continue
                blades.append((round(x, 4), round(y, 4), round(rng.uniform(h_min, h_max), 3),
                               round(rng.uniform(0.0, math.pi), 3),
                               round(rng.uniform(-0.25, 0.25), 3)))
        out.append((ti, blades))
    return out


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
    look = acfg.get('variants', {}).get(apple.variant, DEFAULT_VARIANT)
    bruise = ''
    if 'bruise_color' in look:
        # Dark patch on the top of the apple, the side seen by the camera.
        bruise = f"""
        <visual name="bruise">
          <pose>{r * 0.45} 0 {r * 0.55} 0 0 0</pose>
          <geometry><sphere><radius>{r * 0.6}</radius></sphere></geometry>
          {_material(*look['bruise_color'])}
        </visual>"""
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
          {_material(*look['color'])}
        </visual>{bruise}
      </link>
    </model>"""


def _grass(tree_index, blades, gcfg):
    """Visual-only tuft blades of one tree: the camera sees them, the robot drives through."""
    width = gcfg['blade_width']
    visuals = []
    for k, (x, y, h, yaw, tilt) in enumerate(blades):
        shade = 0.42 + 0.2 * (k % 3) / 2.0
        visuals.append(f"""
        <visual name="blade_{k}">
          <pose>{x} {y} {h / 2} {tilt} 0 {yaw}</pose>
          <geometry><box><size>{width} 0.002 {h}</size></box></geometry>
          {_material(0.15, shade, 0.1)}
        </visual>""")
    return f"""
    <model name="grass_t{tree_index}">
      <static>true</static>
      <link name="link">{''.join(visuals)}
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
    for ti, blades in generate_grass(config, apples):
        parts.append(_grass(ti, blades, config['grass']))
    parts.append('\n  </world>\n</sdf>\n')
    return ''.join(parts)


def build_manifest(config, apples):
    """Ground-truth description of the generated world (for metrics and tests)."""
    entries = []
    dressed = 'variants' in config['apples'] or 'grass' in config
    for a in apples:
        entry = {'name': a.name, 'tree': a.tree_index, 'x': a.x, 'y': a.y}
        if dressed:
            entry.update(variant=a.variant, hidden=a.hidden)
        entries.append(entry)
    return {
        'world_name': config['world_name'],
        'seed': config['seed'],
        'depot': list(config['depot']['position']),
        'trees': [list(t) for t in config['trees']],
        'apples': entries,
    }
