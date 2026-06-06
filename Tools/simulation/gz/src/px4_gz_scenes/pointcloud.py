"""
Analytic surface point-cloud sampler for px4_gz_scenes environments.

Generates world-frame surface points directly from the
:class:`~px4_gz_scenes.scene.Scene` definition -- no Gazebo, no sensor
simulation, no occlusion artifacts.  Useful for:

* Ground-truth occupancy grids and planning benchmarks
* Reproducible tests (deterministic with a seeded RNG)
* Quick iteration when tweaking scenes offline

Each :class:`~px4_gz_scenes.scene_object.SceneObject` contributes points
proportional to its surface area.  Points are sampled uniformly on the
surface of each primitive shape, then transformed to world frame.

Usage::

    from px4_gz_scenes import get_scene, sample_pointcloud

    pc = sample_pointcloud(get_scene("room"), points_per_m2=200.0)
    print(pc.shape, pc.dtype)   # (N, 3) float32
"""

from __future__ import annotations

import math

import numpy as np

from px4_gz_scenes._math import apply_rotation, quat_to_rotation_matrix
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.shapes import Box, Composite, Cylinder, Shape, Sphere


# -- Surface area helpers ----------------------------------------------------


def _surface_area(shape: Shape) -> float:
    """Total surface area in m^2."""
    if isinstance(shape, Box):
        w, d, h = shape.size
        return 2.0 * (w * d + w * h + d * h)

    if isinstance(shape, Cylinder):
        r, h = shape.radius, shape.length
        return 2.0 * math.pi * r * h + 2.0 * math.pi * r * r

    if isinstance(shape, Sphere):
        return 4.0 * math.pi * shape.radius**2

    if isinstance(shape, Composite):
        return sum(_surface_area(child) for child, _, _ in shape.children)

    raise TypeError(f'Unknown shape type: {type(shape)!r}')


# -- Per-shape samplers (local frame) ---------------------------------------
# Each returns an (n, 3) float64 array of surface points centred at the
# shape's local origin.


def _sample_box(shape: Box, n: int, rng: np.random.Generator) -> np.ndarray:
    """Uniformly sample *n* points on the surface of a box."""
    hx, hy, hz = shape.size[0] / 2, shape.size[1] / 2, shape.size[2] / 2
    w, d, h = shape.size

    # Face areas: +/-X (d*h each), +/-Y (w*h each), +/-Z (w*d each).
    areas = np.array([d * h, d * h, w * h, w * h, w * d, w * d])
    probs = areas / areas.sum()
    counts = rng.multinomial(n, probs)

    parts: list[np.ndarray] = []
    for face_idx, cnt in enumerate(counts):
        if cnt == 0:
            continue
        if face_idx < 2:  # +/-X face
            u = rng.uniform(-hy, hy, cnt)
            v = rng.uniform(-hz, hz, cnt)
            x = np.full(cnt, hx if face_idx == 0 else -hx)
            parts.append(np.column_stack([x, u, v]))
        elif face_idx < 4:  # +/-Y face
            u = rng.uniform(-hx, hx, cnt)
            v = rng.uniform(-hz, hz, cnt)
            y = np.full(cnt, hy if face_idx == 2 else -hy)
            parts.append(np.column_stack([u, y, v]))
        else:  # +/-Z face
            u = rng.uniform(-hx, hx, cnt)
            v = rng.uniform(-hy, hy, cnt)
            z = np.full(cnt, hz if face_idx == 4 else -hz)
            parts.append(np.column_stack([u, v, z]))

    if not parts:
        return np.empty((0, 3), dtype=np.float64)
    return np.concatenate(parts, axis=0)


def _sample_cylinder(shape: Cylinder, n: int, rng: np.random.Generator) -> np.ndarray:
    """Uniformly sample *n* points on the surface of a cylinder (Z-aligned)."""
    r, length = shape.radius, shape.length
    hl = length / 2.0

    lateral_area = 2.0 * math.pi * r * length
    cap_area = math.pi * r * r
    areas = np.array([lateral_area, cap_area, cap_area])
    probs = areas / areas.sum()
    counts = rng.multinomial(n, probs)

    parts: list[np.ndarray] = []

    # Lateral surface
    if counts[0] > 0:
        theta = rng.uniform(0.0, 2.0 * math.pi, counts[0])
        z = rng.uniform(-hl, hl, counts[0])
        parts.append(np.column_stack([r * np.cos(theta), r * np.sin(theta), z]))

    # Top and bottom caps
    for cap_idx in (1, 2):
        if counts[cap_idx] == 0:
            continue
        rho = r * np.sqrt(rng.uniform(0.0, 1.0, counts[cap_idx]))
        theta = rng.uniform(0.0, 2.0 * math.pi, counts[cap_idx])
        z_val = hl if cap_idx == 1 else -hl
        parts.append(
            np.column_stack(
                [
                    rho * np.cos(theta),
                    rho * np.sin(theta),
                    np.full(counts[cap_idx], z_val),
                ]
            )
        )

    if not parts:
        return np.empty((0, 3), dtype=np.float64)
    return np.concatenate(parts, axis=0)


def _sample_sphere(shape: Sphere, n: int, rng: np.random.Generator) -> np.ndarray:
    """Uniformly sample *n* points on the surface of a sphere."""
    v = rng.standard_normal((n, 3))
    norms = np.linalg.norm(v, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-12)
    return shape.radius * (v / norms)


def _sample_shape(shape: Shape, n: int, rng: np.random.Generator) -> np.ndarray:
    """Dispatch surface sampling to the appropriate shape helper.

    For Composite shapes, distributes points proportionally to each child's
    surface area, recurses, and transforms by the child's offset and rotation.
    """
    if isinstance(shape, Box):
        return _sample_box(shape, n, rng)

    if isinstance(shape, Cylinder):
        return _sample_cylinder(shape, n, rng)

    if isinstance(shape, Sphere):
        return _sample_sphere(shape, n, rng)

    if isinstance(shape, Composite):
        areas = np.array([_surface_area(child) for child, _, _ in shape.children])
        total = areas.sum()
        if total == 0.0:
            return np.empty((0, 3), dtype=np.float64)
        probs = areas / total
        counts = rng.multinomial(n, probs)

        parts: list[np.ndarray] = []
        for (child_shape, offset, child_rot), cnt in zip(shape.children, counts):
            if cnt == 0:
                continue
            child_pts = _sample_shape(child_shape, int(cnt), rng)
            R_child = quat_to_rotation_matrix(child_rot)
            child_pts = apply_rotation(R_child, child_pts)
            ox, oy, oz = offset
            child_pts += np.array([ox, oy, oz], dtype=np.float64)
            parts.append(child_pts)

        if not parts:
            return np.empty((0, 3), dtype=np.float64)
        return np.concatenate(parts, axis=0)

    raise TypeError(f'Unknown shape type: {type(shape)!r}')


# -- Per-object world-frame transform ----------------------------------------


def _object_points(
    obj: 'SceneObject',  # noqa: F821
    points_per_m2: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Sample surface points for one SceneObject and transform to world frame."""
    area = _surface_area(obj.shape)
    n = max(1, round(area * points_per_m2))
    local_pts = _sample_shape(obj.shape, n, rng)
    R = quat_to_rotation_matrix(obj.rotation)
    world_pts = apply_rotation(R, local_pts)
    px, py, pz = obj.position
    world_pts += np.array([px, py, pz], dtype=np.float64)
    return world_pts


# -- Public API --------------------------------------------------------------


def sample_pointcloud(
    scene: Scene,
    points_per_m2: float = 100.0,
    *,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Sample a surface point cloud from all objects in *scene*.

    Each object contributes points proportional to its surface area.
    Points are uniformly distributed on primitive surfaces (Box, Cylinder,
    Sphere) and transformed to the world frame.

    This is an analytic sampler -- no Gazebo, no sensor noise, no
    occlusion.  Useful for ground-truth occupancy grids, planning
    benchmarks, and reproducible tests.

    Args:
        scene:         The scene to sample from.
        points_per_m2: Target point density (points per square metre of
                       surface area).  Higher values produce denser clouds.
        rng:           NumPy random generator for reproducibility.  Defaults
                       to ``np.random.default_rng()``.

    Returns:
        ``np.ndarray`` of shape ``(N, 3)``, dtype ``float32`` -- world-frame
        XYZ surface points.
    """
    if rng is None:
        rng = np.random.default_rng()

    parts = [_object_points(obj, points_per_m2, rng) for obj in scene.objects]
    if not parts:
        return np.empty((0, 3), dtype=np.float32)
    return np.concatenate(parts, axis=0).astype(np.float32)
