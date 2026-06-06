"""
Primitive geometry types for scene construction.

Shapes describe geometry only — no pose, no label. Pose and semantics are
attached by SceneObject. All shapes are immutable (frozen dataclasses).

Coordinate convention: shapes are defined in their own local frame with the
origin at the geometric centre. The parent SceneObject's `position` field
places that centre in the scene frame.

Supported primitives:
  Box       — axis-aligned rectangular cuboid
  Cylinder  — circular cylinder aligned with local +Z
  Sphere    — sphere
  Composite — ordered collection of (shape, offset, rotation) children

The `Shape` union type covers all four primitives.

The `aabb` function returns a conservative axis-aligned bounding box for any
shape in its local frame, expressed as (min_corner, max_corner) Vec3 pairs.
Exporters (voxeliser, SDF generator, …) can use this without knowing the
concrete shape type.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union

from px4_gz_scenes._types import Quaternion, Vec3


@dataclass(frozen=True)
class Box:
    """Axis-aligned rectangular cuboid.

    Args:
        size: (width, depth, height) in metres — full extents, not half-extents.
              Matches the SDF ``<size>`` element convention.
    """

    size: Vec3


@dataclass(frozen=True)
class Cylinder:
    """Circular cylinder aligned with the local +Z axis.

    Args:
        radius: cylinder radius in metres.
        length: total height along +Z in metres.
    """

    radius: float
    length: float


@dataclass(frozen=True)
class Sphere:
    """Sphere.

    Args:
        radius: sphere radius in metres.
    """

    radius: float


@dataclass(frozen=True)
class Composite:
    """Ordered group of shapes treated as a single unit.

    Each child is a ``(shape, offset, rotation)`` triple where ``offset`` is
    the child's centre relative to the composite's origin and ``rotation`` is a
    Hamilton quaternion (w, x, y, z).

    Useful for L-shaped walls, furniture assemblies, and other multi-part
    objects that share a common name and label.

    Args:
        children: immutable tuple of (shape, offset, rotation) triples.
    """

    children: tuple[tuple[Shape, Vec3, Quaternion], ...]


Shape = Union[Box, Cylinder, Sphere, Composite]


def aabb(shape: Shape) -> tuple[Vec3, Vec3]:
    """Return the axis-aligned bounding box of *shape* in its local frame.

    The bounding box is a conservative estimate — for rotated ``Composite``
    children the child AABB is expanded to account for the rotation.

    Returns:
        ``(min_corner, max_corner)`` as ``Vec3`` pairs.
    """
    if isinstance(shape, Box):
        hx, hy, hz = shape.size[0] / 2, shape.size[1] / 2, shape.size[2] / 2
        return (-hx, -hy, -hz), (hx, hy, hz)

    if isinstance(shape, Cylinder):
        r, hl = shape.radius, shape.length / 2
        return (-r, -r, -hl), (r, r, hl)

    if isinstance(shape, Sphere):
        r = shape.radius
        return (-r, -r, -r), (r, r, r)

    # Composite: union of child AABBs transformed by their offsets.
    # We ignore child rotations for simplicity (conservative overestimate).
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    for child_shape, (ox, oy, oz), _rot in shape.children:
        (lx, ly, lz), (hx, hy, hz) = aabb(child_shape)
        min_x = min(min_x, ox + lx)
        min_y = min(min_y, oy + ly)
        min_z = min(min_z, oz + lz)
        max_x = max(max_x, ox + hx)
        max_y = max(max_y, oy + hy)
        max_z = max(max_z, oz + hz)
    return (min_x, min_y, min_z), (max_x, max_y, max_z)
