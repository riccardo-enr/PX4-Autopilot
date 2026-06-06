"""
Visualise a px4_gz_scenes environment in 3D.

Usage:
    python scripts/visualise_scene.py            # defaults to 'room'
    python scripts/visualise_scene.py office
    python scripts/visualise_scene.py room
"""

import sys
from px4_gz_scenes import get_scene, list_scenes, visualise_scene

if __name__ == '__main__':
    name = sys.argv[1] if len(sys.argv) > 1 else 'room'

    if name not in list_scenes():
        print(f'Unknown scene {name!r}. Available: {list_scenes()}')
        sys.exit(1)

    scene = get_scene(name)
    print(f"Rendering '{scene.name}' — {len(scene.objects)} objects")
    visualise_scene(scene)
