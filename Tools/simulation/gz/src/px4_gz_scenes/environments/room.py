"""
Room environment — a 15 × 15 × 3 m indoor room.

Geometry (all coordinates in ENU, metres):

    ┌─────────────────────────────────────┐  z = 3.0 m (ceiling)
    │                                     │
    │    [col]     ║  <wall B>             │
    │              ║                      │
    │  [table A]   ║                      │
    │              ║       [table B]      │
    │  <wall A>    ║                      │
    │                                     │
    └─────────────────────────────────────┘  z = 0.0 m (floor)

Objects:
  - floor slab        z = 0.0 → 0.2 m
  - ceiling slab      z = 2.8 → 3.0 m
  - 4 boundary walls  full height, 0.2 m thick
  - wall A            x ≈ 4.2, y = 3–7 m,    z = 0–3 m  (partition)
  - wall B            x ≈ 8.2, y = 6–11 m,   z = 0–3 m  (partition)
  - table A           x = 6–7.5, y = 2–3.5 m,  z = 0.8–1.2 m  (table)
  - table B           x = 10–11.5, y = 9–10.5 m, z = 0.8–1.2 m (table)
  - column            x = 11–12.5, y = 4–5.5 m, z = 0–3 m    (column)

Partitions are full-height so the UAV must route around them through gaps.
Tables remain at table height (0.8–1.2 m), producing some frontier clusters
at different z-levels.
Start position (2, 2, 1.5) is kept clear — all interior obstacles start
at x ≥ 4.0.
"""

from __future__ import annotations

from px4_gz_scenes.registry import register_scene
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import (
    SceneObject,
    LABEL_COLUMN,
    LABEL_PARTITION,
    LABEL_TABLE,
)
from px4_gz_scenes.shapes import Box


def _box_obj(
    name: str,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    label: str,
) -> SceneObject:
    """Create a SceneObject from corner coordinates (convenience helper)."""
    return SceneObject(
        name=name,
        shape=Box(size=(x1 - x0, y1 - y0, z1 - z0)),
        position=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
        label=label,
    )


@register_scene('room')
def make_room(
    ext_x: float = 15.0,
    ext_y: float = 15.0,
    z_height: float = 3.0,
    wall_thickness: float = 0.2,
) -> Scene:
    """Build the room scene.

    Args:
        ext_x:          Room length along X (East) in metres.
        ext_y:          Room length along Y (North) in metres.
        z_height:       Room height in metres.
        wall_thickness: Thickness of boundary walls and floor/ceiling slabs.

    Returns:
        A :class:`Scene` containing all room objects.
    """
    scene = Scene(name='room', extent=(ext_x, ext_y, z_height))
    scene.add_boundary(wall_thickness=wall_thickness, slab_thickness=wall_thickness)

    # Wall A — left-mid partition (full-height)
    scene.add(_box_obj('wall_a', 4.0, 4.4, 3.0, 7.0, 0.0, z_height, LABEL_PARTITION))

    # Wall B — right-mid partition (full-height)
    scene.add(_box_obj('wall_b', 8.0, 8.4, 6.0, 11.0, 0.0, z_height, LABEL_PARTITION))

    # Table A
    scene.add(_box_obj('table_a', 6.0, 7.5, 2.0, 3.5, 0.8, 1.2, LABEL_TABLE))

    # Table B
    scene.add(_box_obj('table_b', 10.0, 11.5, 9.0, 10.5, 0.8, 1.2, LABEL_TABLE))

    # Full-height structural column
    scene.add(_box_obj('column', 11.0, 12.5, 4.0, 5.5, 0.0, z_height, LABEL_COLUMN))

    return scene
