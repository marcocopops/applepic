import math
import os
import xml.etree.ElementTree as ET

import pytest

from applepic_gazebo.orchard import build_manifest, build_world_sdf, generate_apples, load_config

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'orchard.yaml')


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
