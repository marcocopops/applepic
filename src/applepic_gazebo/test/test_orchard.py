import math
import os
import xml.etree.ElementTree as ET

import pytest

from applepic_gazebo.orchard import (
    build_manifest, build_world_sdf, generate_apples, generate_grass, load_config,
    obstacle_clearance)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'orchard.yaml')
HARD_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'orchard_hard.yaml')


@pytest.fixture
def config():
    return load_config(CONFIG)


def test_apples_deterministic(config):
    assert generate_apples(config) == generate_apples(config)
    other = dict(config, seed=config['seed'] + 1)
    assert generate_apples(other) != generate_apples(config)


def test_apples_count_and_ring(config):
    apples = generate_apples(config)
    n_min, n_max = config['apples']['per_tree']
    r_min, r_max = config['apples']['ring']
    for ti, (tx, ty) in enumerate(config['trees']):
        mine = [a for a in apples if a.tree_index == ti]
        assert n_min <= len(mine) <= n_max
        for a in mine:
            assert r_min - 1e-3 <= math.hypot(a.x - tx, a.y - ty) <= r_max + 1e-3


def test_apples_spacing_and_unique_names(config):
    apples = generate_apples(config)
    assert len({a.name for a in apples}) == len(apples)
    for i, a in enumerate(apples):
        for b in apples[i + 1:]:
            assert math.hypot(a.x - b.x, a.y - b.y) >= config['apples']['min_spacing'] - 1e-3


def test_world_sdf_is_valid_xml(config):
    apples = generate_apples(config)
    root = ET.fromstring(build_world_sdf(config, apples))
    world = root.find('world')
    assert world.get('name') == config['world_name']
    names = [m.get('name') for m in world.findall('model')]
    assert 'ground' in names and 'depot' in names
    assert sum(n.startswith('tree_') for n in names) == len(config['trees'])
    assert sum(n.startswith('apple_') for n in names) == len(apples)


def test_manifest_matches(config):
    apples = generate_apples(config)
    manifest = build_manifest(config, apples)
    assert len(manifest['apples']) == len(apples)
    assert manifest['trees'] == [list(t) for t in config['trees']]


def test_impossible_placement_raises(config):
    config['apples']['per_tree'] = [500, 500]
    with pytest.raises(ValueError):
        generate_apples(config)


def test_hard_world_keeps_positions_and_adds_grass():
    plain, hard = load_config(CONFIG), load_config(HARD_CONFIG)
    assert plain['trees'] == hard['trees']
    plain_apples, hard_apples = generate_apples(plain), generate_apples(hard)
    assert [(a.x, a.y) for a in plain_apples] == [(a.x, a.y) for a in hard_apples]
    assert {a.variant for a in hard_apples} > {'ripe'}
    assert any(a.hidden for a in hard_apples)
    assert hard_apples == generate_apples(hard)

    world = ET.fromstring(build_world_sdf(hard, hard_apples)).find('world')
    grass = [m for m in world.findall('model') if m.get('name').startswith('grass_')]
    assert len(grass) == len(hard['trees'])
    for model in grass:   # the robot must drive through the grass
        assert model.find('link').findall('visual') and not model.find('link').findall('collision')
    manifest = build_manifest(hard, hard_apples)
    assert {'variant', 'hidden'} <= set(manifest['apples'][0])


def test_grass_stays_clear_of_the_trunks():
    hard = load_config(HARD_CONFIG)
    for ti, blades in generate_grass(hard, generate_apples(hard)):
        tx, ty = hard['trees'][ti]
        assert all(math.hypot(b[0] - tx, b[1] - ty) >= hard['grass']['trunk_clearance']
                   for b in blades)


def test_plain_world_has_no_grass(config):
    assert generate_grass(config, generate_apples(config)) == []
    manifest = build_manifest(config, generate_apples(config))
    assert 'variant' not in manifest['apples'][0]


def _obstacle_config():
    return load_config(os.path.join(os.path.dirname(CONFIG), 'orchard_obstacles.yaml'))


def test_obstacle_world_keeps_positions_and_adds_static_boxes(config):
    hard = _obstacle_config()
    apples = generate_apples(hard)
    assert apples == generate_apples(config)
    models = ET.fromstring(build_world_sdf(hard, apples)).find('world').findall('model')
    obstacles = [m for m in models if m.get('name').startswith('obstacle_')]
    assert len(obstacles) == len(hard['obstacles'])
    assert all(m.find('static').text == 'true' for m in obstacles)


def test_obstacle_clearance_follows_the_yaw():
    log = {'position': [0.0, 0.0], 'size': [2.0, 0.2, 0.2], 'yaw': math.pi / 2}
    assert obstacle_clearance(log, (0.0, 0.9)) == 0.0
    assert obstacle_clearance(log, (0.9, 0.0)) == pytest.approx(0.8)


def test_obstacle_on_an_apple_raises(config):
    apples = generate_apples(config)
    bad = dict(config, obstacles=[{'position': [apples[0].x, apples[0].y], 'size': [0.4, 0.4, 0.4]}])
    with pytest.raises(ValueError):
        build_world_sdf(bad, apples)
