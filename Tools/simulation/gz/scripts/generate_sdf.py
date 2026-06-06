"""
generate_sdf.py -- export one or all registered scenes to Gazebo SDF world files.

Usage:
    python scripts/generate_sdf.py                   # export all scenes
    python scripts/generate_sdf.py --scene office    # export a single scene
    python scripts/generate_sdf.py --out-dir /tmp    # custom output directory

Output files are written to extern/px4_gz_scenes/worlds/ by default, named
after the scene (e.g. office.sdf, room.sdf).

Pass --no-ceiling to omit the ceiling slab so the scene interior is visible
when inspecting in Gazebo.
"""

import argparse
import sys
from pathlib import Path

# Allow running from repo root without installing the package.
_REPO_SRC = Path(__file__).resolve().parent.parent / 'src'
if str(_REPO_SRC) not in sys.path:
    sys.path.insert(0, str(_REPO_SRC))

from px4_gz_scenes import get_scene, list_scenes, scene_to_sdf
from px4_gz_scenes.scene_object import LABEL_CEILING


def main() -> None:
    parser = argparse.ArgumentParser(description='Export px4_gz_scenes to Gazebo SDF worlds.')
    parser.add_argument(
        '--scene',
        metavar='NAME',
        help='Name of the scene to export (default: all registered scenes).',
    )
    parser.add_argument(
        '--out-dir',
        metavar='DIR',
        default=str(Path(__file__).resolve().parent.parent / 'worlds'),
        help='Directory to write .sdf files (default: worlds/).',
    )
    parser.add_argument(
        '--no-ceiling',
        action='store_true',
        help='Omit the ceiling slab so the scene is visible from above in Gazebo.',
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    exclude = [LABEL_CEILING] if args.no_ceiling else None

    scenes_to_export = [args.scene] if args.scene else list_scenes()

    for name in scenes_to_export:
        scene = get_scene(name)
        sdf_text = scene_to_sdf(scene, exclude_labels=exclude)
        out_path = out_dir / f'{name}.sdf'
        out_path.write_text(sdf_text, encoding='utf-8')
        n_obj = len(scene.objects)
        print(f'Wrote {out_path}  ({n_obj} objects, extent {scene.extent})')


if __name__ == '__main__':
    main()
