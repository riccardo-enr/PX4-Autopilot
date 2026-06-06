"""
Scene — a named 3-D environment as an ordered collection of SceneObjects.

A Scene is the top-level container produced by every environment factory
function. It is intentionally mutable so that builder functions can
``add()`` objects incrementally, matching the ergonomics of the original
``fill_box()`` approach without coupling to a voxel representation.

Boundary helpers (``add_floor``, ``add_ceiling``, ``add_walls``,
``add_boundary``) compute object dimensions from ``self.extent`` so the
caller does not have to repeat those calculations in every environment.

The ``resolution`` field is a *hint* for downstream exporters: the suggested
voxel / grid cell size in metres. It is not used by the Scene itself.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from px4_gz_scenes._types import Vec3
from px4_gz_scenes.scene_object import (
    SceneObject,
    LABEL_CEILING,
    LABEL_FLOOR,
    LABEL_WALL,
)
from px4_gz_scenes.shapes import Box


@dataclass
class Scene:
    """A complete 3-D environment as an ordered list of SceneObjects.

    Args:
        name:       Human-readable identifier (e.g. ``"room"``).
        extent:     Outer bounding box ``(x_size, y_size, z_size)`` in metres.
                    Defines the coordinate domain: objects should stay within
                    ``[0, x_size] x [0, y_size] x [0, z_size]``.
        objects:    Ordered list of scene objects; mutated by ``add()``.
        resolution: Suggested discretisation step for exporters (metres).
        frame:      Coordinate frame label — ``"ENU"`` by default.
        origin:     World-frame position of the scene's ``(0, 0, 0)`` corner.
    """

    name: str
    extent: Vec3
    objects: list[SceneObject] = field(default_factory=list)
    resolution: float = 0.2
    frame: str = 'ENU'
    origin: Vec3 = (0.0, 0.0, 0.0)

    # ── Mutation helpers ────────────────────────────────────────────────────

    def add(self, obj: SceneObject) -> None:
        """Append *obj* to the scene."""
        self.objects.append(obj)

    def add_floor(self, thickness: float = 0.2, label: str = LABEL_FLOOR) -> None:
        """Add a floor slab centred at z = thickness / 2."""
        ex, ey, _ = self.extent
        self.add(
            SceneObject(
                name='floor',
                shape=Box(size=(ex, ey, thickness)),
                position=(ex / 2, ey / 2, thickness / 2),
                label=label,
            )
        )

    def add_ceiling(self, thickness: float = 0.2, label: str = LABEL_CEILING) -> None:
        """Add a ceiling slab centred at z = z_extent - thickness / 2."""
        ex, ey, ez = self.extent
        self.add(
            SceneObject(
                name='ceiling',
                shape=Box(size=(ex, ey, thickness)),
                position=(ex / 2, ey / 2, ez - thickness / 2),
                label=label,
            )
        )

    def add_walls(self, thickness: float = 0.2, label: str = LABEL_WALL) -> None:
        """Add four full-height boundary walls (south, north, west, east)."""
        ex, ey, ez = self.extent
        t = thickness
        walls = [
            # name,        size,              position
            ('wall_south', (ex, t, ez), (ex / 2, t / 2, ez / 2)),
            ('wall_north', (ex, t, ez), (ex / 2, ey - t / 2, ez / 2)),
            ('wall_west', (t, ey, ez), (t / 2, ey / 2, ez / 2)),
            ('wall_east', (t, ey, ez), (ex - t / 2, ey / 2, ez / 2)),
        ]
        for name, size, pos in walls:
            self.add(
                SceneObject(
                    name=name,
                    shape=Box(size=size),
                    position=pos,
                    label=label,
                )
            )

    def add_boundary(
        self,
        wall_thickness: float = 0.2,
        slab_thickness: float = 0.2,
    ) -> None:
        """Add floor, ceiling, and four boundary walls in one call."""
        self.add_floor(thickness=slab_thickness)
        self.add_ceiling(thickness=slab_thickness)
        self.add_walls(thickness=wall_thickness)

    # ── Query helpers ───────────────────────────────────────────────────────

    def filter_by_label(self, label: str) -> list[SceneObject]:
        """Return all objects whose label matches *label* exactly."""
        return [o for o in self.objects if o.label == label]

    def __repr__(self) -> str:
        return (
            f'Scene(name={self.name!r}, extent={self.extent}, '
            f'objects={len(self.objects)}, resolution={self.resolution})'
        )
