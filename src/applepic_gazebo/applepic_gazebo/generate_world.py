"""CLI: generate the orchard SDF world and its apple manifest from a YAML config."""
import argparse
import os

import yaml

from applepic_gazebo.orchard import build_manifest, build_world_sdf, generate_apples, load_config


def _default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('applepic_gazebo'), 'config', 'orchard.yaml')
    except Exception:  # noqa: BLE001 - fall back to the source tree layout
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'config', 'orchard.yaml')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=_default_config())
    parser.add_argument('--output', default='orchard.sdf', help='SDF output path')
    parser.add_argument('--manifest', default=None,
                        help='apple manifest output (default: <output>_apples.yaml)')
    parser.add_argument('--seed', type=int, default=None, help='override config seed')
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.seed is not None:
        config['seed'] = args.seed
    apples = generate_apples(config)
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(build_world_sdf(config, apples))
    manifest_path = args.manifest or os.path.splitext(args.output)[0] + '_apples.yaml'
    with open(manifest_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(build_manifest(config, apples), f, sort_keys=False)
    print(f'wrote {args.output} ({len(config["trees"])} trees, {len(apples)} apples) '
          f'and {manifest_path}')


if __name__ == '__main__':
    main()
