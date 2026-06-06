"""
3-D occupancy grid exporter for px4_gz_scenes environments.

Converts a :class:`~px4_gz_scenes.scene.Scene` into a dense boolean NumPy
array of shape ``(Nx, Ny, Nz)`` where ``True`` means the voxel centre falls
inside at least one :class:`~px4_gz_scenes.scene_object.SceneObject`.

Grid parameters are derived from the scene:

* **cell size** — ``scene.resolution`` (metres)
* **grid dimensions** — ``Ni = ceil(scene.extent[i] / resolution)``
* **voxel centre** — ``((i + 0.5) * res, (j + 0.5) * res, (k + 0.5) * res)``

The implementation is fully vectorised using NumPy broadcasting; there is no
Python triple-loop over voxels.  For a 15 × 15 × 3 m room at 0.2 m resolution
the working array is ~20 MB — well within memory budgets for interactive use.

Usage::

    import numpy as np
    from px4_gz_scenes import get_scene
    from px4_gz_scenes.occupancy import to_occupancy_grid

    grid = to_occupancy_grid(get_scene("room"))
    print(grid.shape, grid.sum())   # (75, 75, 15)  <occupied count>
    np.save("room_occ.npy", grid)
"""

from __future__ import annotations

import math

import numpy as np

from px4_gz_scenes._math import apply_rotation, quat_to_rotation_matrix
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import SceneObject
from px4_gz_scenes.shapes import Box, Composite, Cylinder, Shape, Sphere, aabb


# ── Per-shape containment helpers ───────────────────────────────────────────
# All helpers receive *local_pts* of shape (Nx, Ny, Nz, 3) already expressed
# in the shape's local frame (origin at the shape centre).  They return a
# boolean mask of the same leading shape (Nx, Ny, Nz).


def _inside_box(local_pts: np.ndarray, shape: Box) -> np.ndarray:
    hx, hy, hz = shape.size[0] / 2, shape.size[1] / 2, shape.size[2] / 2
    return (
        (np.abs(local_pts[..., 0]) <= hx)
        & (np.abs(local_pts[..., 1]) <= hy)
        & (np.abs(local_pts[..., 2]) <= hz)
    )


def _inside_cylinder(local_pts: np.ndarray, shape: Cylinder) -> np.ndarray:
    r2 = local_pts[..., 0] ** 2 + local_pts[..., 1] ** 2
    return (r2 <= shape.radius**2) & (np.abs(local_pts[..., 2]) <= shape.length / 2)


def _inside_sphere(local_pts: np.ndarray, shape: Sphere) -> np.ndarray:
    d2 = np.sum(local_pts**2, axis=-1)
    return d2 <= shape.radius**2


def _inside_shape(local_pts: np.ndarray, shape: Shape) -> np.ndarray:
    """Dispatch containment test to the appropriate shape helper.

    For :class:`~px4_gz_scenes.shapes.Composite` shapes the function recurses
    into each child, shifting and rotating ``local_pts`` by the child's offset
    and rotation before testing.
    """
    if isinstance(shape, Box):
        return _inside_box(local_pts, shape)

    if isinstance(shape, Cylinder):
        return _inside_cylinder(local_pts, shape)

    if isinstance(shape, Sphere):
        return _inside_sphere(local_pts, shape)

    if isinstance(shape, Composite):
        mask = np.zeros(local_pts.shape[:-1], dtype=bool)
        for child_shape, (ox, oy, oz), child_rot in shape.children:
            child_local = local_pts - np.array([ox, oy, oz], dtype=np.float64)
            R_child = quat_to_rotation_matrix(child_rot)
            # Inverse rotation: apply R_child.T (= R_child^{-1} for unit quat)
            child_local = child_local @ R_child  # pts @ R.T where we pass R.T as R_child
            mask |= _inside_shape(child_local, child_shape)
        return mask

    raise TypeError(f'Unknown shape type: {type(shape)!r}')


def _mark_object(
    grid: np.ndarray,
    obj: SceneObject,
    res: float,
) -> None:
    """OR-accumulate occupied voxels for one SceneObject into *grid*.

    Only voxels within the object's world-frame AABB are tested, which gives
    large speedups for thin objects (floors, ceilings, walls) that would
    otherwise redundantly test the entire grid volume.
    """
    nx, ny, nz = grid.shape

    # Compute the rotation matrix once; reuse for both AABB expansion and
    # the local-frame transform below.
    R = quat_to_rotation_matrix(obj.rotation)

    # Rotate the 8 local AABB corners to world frame and take their envelope.
    lo, hi = aabb(obj.shape)
    corners = np.array(
        [
            [lo[0], lo[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [lo[0], hi[1], lo[2]],
            [lo[0], hi[1], hi[2]],
            [hi[0], lo[1], lo[2]],
            [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], lo[2]],
            [hi[0], hi[1], hi[2]],
        ],
        dtype=np.float64,
    )
    rotated = corners @ R.T  # (8, 3) in world frame (no translation yet)
    pos = np.array(obj.position, dtype=np.float64)
    world_lo = rotated.min(axis=0) + pos
    world_hi = rotated.max(axis=0) + pos

    # Convert world AABB to grid index ranges, clipped to grid bounds.
    i_min = max(0, int(math.floor(world_lo[0] / res)))
    j_min = max(0, int(math.floor(world_lo[1] / res)))
    k_min = max(0, int(math.floor(world_lo[2] / res)))
    i_max = min(nx, int(math.ceil(world_hi[0] / res)))
    j_max = min(ny, int(math.ceil(world_hi[1] / res)))
    k_max = min(nz, int(math.ceil(world_hi[2] / res)))

    if i_min >= i_max or j_min >= j_max or k_min >= k_max:
        return  # Object fully outside grid

    # Build voxel centres only for the sub-volume within the AABB.
    ii = (np.arange(i_min, i_max, dtype=np.float64) + 0.5) * res
    jj = (np.arange(j_min, j_max, dtype=np.float64) + 0.5) * res
    kk = (np.arange(k_min, k_max, dtype=np.float64) + 0.5) * res
    Xi, Yj, Zk = np.meshgrid(ii, jj, kk, indexing='ij')
    sub_pts = np.stack([Xi, Yj, Zk], axis=-1)

    # Transform sub-volume to object-local frame and test containment.
    local_pts = sub_pts - pos
    local_pts = apply_rotation(R.T, local_pts)
    grid[i_min:i_max, j_min:j_max, k_min:k_max] |= _inside_shape(local_pts, obj.shape)


# ── Public API ───────────────────────────────────────────────────────────────


def to_occupancy_grid(scene: Scene) -> np.ndarray:
    """Return a 3-D occupancy grid for *scene*.

    Args:
        scene: The scene to voxelise.  Uses ``scene.resolution`` for cell size
               and ``scene.extent`` for grid dimensions.

    Returns:
        Boolean NumPy array of shape ``(Nx, Ny, Nz)`` where index ``(i, j, k)``
        is ``True`` if the voxel centre
        ``((i + 0.5) * res, (j + 0.5) * res, (k + 0.5) * res)``
        is inside at least one :class:`~px4_gz_scenes.scene_object.SceneObject`.
    """
    res = scene.resolution
    nx = math.ceil(scene.extent[0] / res)
    ny = math.ceil(scene.extent[1] / res)
    nz = math.ceil(scene.extent[2] / res)

    grid = np.zeros((nx, ny, nz), dtype=bool)
    for obj in scene.objects:
        _mark_object(grid, obj, res)

    return grid
