#!/usr/bin/env python3
"""
export_occupancy.py -- export a px4_gz_scenes environment to a .npy occupancy grid.

Usage:
    python scripts/export_occupancy.py room
    python scripts/export_occupancy.py office --output office_occ.npy
    python scripts/export_occupancy.py room --resolution 0.1
"""

import argparse
import sys
from pathlib import Path

# Allow running from repo root without installing the package.
_REPO_SRC = Path(__file__).resolve().parent.parent / 'src'
if str(_REPO_SRC) not in sys.path:
    sys.path.insert(0, str(_REPO_SRC))

import numpy as np

from px4_gz_scenes import get_scene, list_scenes, to_occupancy_grid


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Export a px4_gz_scenes environment to a .npy occupancy grid.'
    )
    parser.add_argument('scene', metavar='SCENE', help='Name of the scene to export.')
    parser.add_argument(
        '--output',
        '-o',
        metavar='PATH',
        default=None,
        help='Output .npy file path (default: <scene>_occ.npy).',
    )
    parser.add_argument(
        '--resolution',
        '-r',
        metavar='RES',
        type=float,
        default=None,
        help='Voxel cell size in metres (default: scene.resolution).',
    )
    args = parser.parse_args()

    if args.scene not in list_scenes():
        print(f'Unknown scene {args.scene!r}. Available: {list_scenes()}')
        sys.exit(1)

    scene = get_scene(args.scene)

    if args.resolution is not None:
        scene.resolution = args.resolution

    output = Path(args.output) if args.output else Path(f'{args.scene}_occ.npy')

    grid = to_occupancy_grid(scene)
    np.save(output, grid)

    occupied = int(grid.sum())
    print(f'Saved {grid.shape} grid ({occupied} occupied voxels) -> {output}')


if __name__ == '__main__':
    main()
