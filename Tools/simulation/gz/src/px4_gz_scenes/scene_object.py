"""
SceneObject — a positioned, labelled shape in a scene.

A SceneObject combines:
  - a geometric primitive (Shape)
  - a pose in the scene frame (position + rotation)
  - a semantic label (plain string, open vocabulary)
  - an optional RGBA colour hint for visualisation

Label constants below document the conventional vocabulary used by the
bundled environment definitions. They are not enforced; any string is valid.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from px4_gz_scenes._types import Color, IDENTITY_QUAT, Quaternion, Vec3
from px4_gz_scenes.shapes import Shape

# ── Semantic label constants ────────────────────────────────────────────────
LABEL_WALL = 'wall'
LABEL_FLOOR = 'floor'
LABEL_CEILING = 'ceiling'
LABEL_TABLE = 'table'
LABEL_COLUMN = 'column'
LABEL_PARTITION = 'partition'
LABEL_RACK = 'rack'
LABEL_DOOR = 'door'
LABEL_OBSTACLE = 'obstacle'


@dataclass(frozen=True)
class SceneObject:
    """A positioned, labelled shape in a 3-D scene.

    Args:
        name:     Unique identifier within the scene (e.g. ``"wall_north"``).
        shape:    Geometric primitive describing the object's extent.
        position: Centre of the shape in the scene's world frame (metres, ENU).
        rotation: Orientation as a Hamilton unit quaternion (w, x, y, z).
                  Defaults to identity (no rotation).
        label:    Semantic category string. Use the ``LABEL_*`` constants for
                  the standard vocabulary.
        color:    Optional RGBA hint for visualisation, each channel in [0, 1].
    """

    name: str
    shape: Shape
    position: Vec3 = (0.0, 0.0, 0.0)
    rotation: Quaternion = field(default=IDENTITY_QUAT)
    label: str = LABEL_OBSTACLE
    color: Color | None = None
