"""
Office environment — a 15 × 12 × 3 m open-plan office.

Geometry (all coordinates in ENU, metres):

  x →
  ┌─────────────────────────────────────────────┐  y = 12
  │  [col A]      ║    ║     [col B]            │
  │               ║    ║                        │
  │  [desk 2]   ══╩════╝                        │
  │               ║         [desk 4]            │
  │  [desk 1]     ║         [desk 3]  [rack]    │
  │               ║                             │
  └─────────────────────────────────────────────┘  y = 0
  x=0                                          x=15

Cubicle partitions (full-height, z = 0 → 3 m):
  Row A   x ≈ 4.8–5.2, gaps at y = 5.2–6.8  (passage, 1.6 m)
  Row B   x ≈ 8.8–9.2, gaps at y = 5.7–7.3  (passage, 1.6 m)
  Connector  x = 5.2–8.8, y ≈ 6.8–7.2       (transverse connector)

Desks (z = 0.7 → 1.0 m, table height):
  desk_1  x = 5.5–7.0,  y = 4.5–5.5
  desk_2  x = 7.0–8.5,  y = 8.5–9.5
  desk_3  x = 10.5–12.0, y = 4.0–5.0
  desk_4  x = 10.5–12.0, y = 7.5–8.5

Full-height columns (z = 0 → 3 m):
  col_a  x = 6.5–7.0, y = 9.5–10.0
  col_b  x = 11.5–12.0, y = 9.5–10.0

Server rack (z = 0 → 2.0 m):
  rack  x = 12.5–13.5, y = 4.5–5.5

Start position (2, 2, 1.5) is kept clear — all interior obstacles start
at x ≥ 4.8 or y ≥ 3.0.
"""

from __future__ import annotations

from px4_gz_scenes.registry import register_scene
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import (
    SceneObject,
    LABEL_COLUMN,
    LABEL_PARTITION,
    LABEL_RACK,
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


@register_scene('office')
def make_office(
    ext_x: float = 15.0,
    ext_y: float = 12.0,
    z_height: float = 3.0,
    wall_thickness: float = 0.2,
) -> Scene:
    """Build the office scene.

    Args:
        ext_x:          Room length along X (East) in metres.
        ext_y:          Room length along Y (North) in metres.
        z_height:       Room height in metres.
        wall_thickness: Thickness of boundary walls and floor/ceiling slabs.

    Returns:
        A :class:`Scene` containing all office objects.
    """
    scene = Scene(name='office', extent=(ext_x, ext_y, z_height))
    scene.add_boundary(wall_thickness=wall_thickness, slab_thickness=wall_thickness)

    # ── Cubicle row A (x ≈ 5) with passage at y = 5.2–6.8 ─────────────────
    scene.add(_box_obj('partition_a_lower', 4.8, 5.2, 4.0, 5.2, 0.0, z_height, LABEL_PARTITION))
    scene.add(_box_obj('partition_a_upper', 4.8, 5.2, 6.8, 8.0, 0.0, z_height, LABEL_PARTITION))

    # ── Cubicle row B (x ≈ 9) with passage at y = 5.7–7.3 ─────────────────
    scene.add(_box_obj('partition_b_lower', 8.8, 9.2, 3.0, 5.7, 0.0, z_height, LABEL_PARTITION))
    scene.add(_box_obj('partition_b_upper', 8.8, 9.2, 7.3, 9.0, 0.0, z_height, LABEL_PARTITION))

    # ── Transverse connector between row A and row B ────────────────────────
    scene.add(_box_obj('partition_connector', 5.2, 8.8, 6.8, 7.2, 0.0, z_height, LABEL_PARTITION))

    # ── Desks ───────────────────────────────────────────────────────────────
    scene.add(_box_obj('desk_1', 5.5, 7.0, 4.5, 5.5, 0.7, 1.0, LABEL_TABLE))
    scene.add(_box_obj('desk_2', 7.0, 8.5, 8.5, 9.5, 0.7, 1.0, LABEL_TABLE))
    scene.add(_box_obj('desk_3', 10.5, 12.0, 4.0, 5.0, 0.7, 1.0, LABEL_TABLE))
    scene.add(_box_obj('desk_4', 10.5, 12.0, 7.5, 8.5, 0.7, 1.0, LABEL_TABLE))

    # ── Full-height structural columns ──────────────────────────────────────
    scene.add(_box_obj('col_a', 6.5, 7.0, 9.5, 10.0, 0.0, z_height, LABEL_COLUMN))
    scene.add(_box_obj('col_b', 11.5, 12.0, 9.5, 10.0, 0.0, z_height, LABEL_COLUMN))

    # ── Server rack ─────────────────────────────────────────────────────────
    scene.add(_box_obj('rack', 12.5, 13.5, 4.5, 5.5, 0.0, 2.0, LABEL_RACK))

    return scene
