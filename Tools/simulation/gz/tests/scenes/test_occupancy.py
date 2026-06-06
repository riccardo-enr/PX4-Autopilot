import math
import pytest
from px4_gz_scenes import get_scene
from px4_gz_scenes._types import IDENTITY_QUAT, euler_to_quat
from px4_gz_scenes.occupancy import to_occupancy_grid
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import SceneObject
from px4_gz_scenes.shapes import Box, Composite, Cylinder, Sphere


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture
def single_box_scene():
    """2×2×2 box centred at (1,1,1) inside a 2×2×2 extent, resolution 0.5."""
    scene = Scene(name='single_box', extent=(2.0, 2.0, 2.0), resolution=0.5)
    scene.add(SceneObject(name='box', shape=Box(size=(1.0, 1.0, 1.0)), position=(1.0, 1.0, 1.0)))
    return scene


# ── Tests ────────────────────────────────────────────────────────────────────


def test_grid_shape_matches_extent():
    scene = Scene(name='s', extent=(4.0, 4.0, 2.0), resolution=0.5)
    grid = to_occupancy_grid(scene)
    assert grid.shape == (8, 8, 4)


def test_empty_scene_all_false():
    scene = Scene(name='empty', extent=(2.0, 2.0, 2.0), resolution=0.5)
    grid = to_occupancy_grid(scene)
    assert grid.dtype == bool
    assert not grid.any()


def test_single_box_centre_voxel_occupied(single_box_scene):
    grid = to_occupancy_grid(single_box_scene)
    # Box (size 1) centred at (1,1,1) with res=0.5: voxels (1,1,1) and (2,2,2) are inside.
    assert grid[1, 1, 1]


def test_single_box_far_voxel_unoccupied(single_box_scene):
    grid = to_occupancy_grid(single_box_scene)
    # Voxel (0,0,0) centre at (0.25, 0.25, 0.25) is outside the box.
    assert not grid[0, 0, 0]


def test_sphere_centre_occupied():
    scene = Scene(name='sphere', extent=(4.0, 4.0, 4.0), resolution=0.5)
    scene.add(SceneObject(name='s', shape=Sphere(radius=0.6), position=(2.0, 2.0, 2.0)))
    grid = to_occupancy_grid(scene)
    # Voxel index (3,3,3) has centre (1.75,1.75,1.75)? No — let's compute properly.
    # Centre of voxel (i,j,k) = (i+0.5)*0.5. Position 2.0 → index 4 → centre 2.25.
    # Voxel (3,3,3) centre = (1.75,1.75,1.75) — distance from (2,2,2) = sqrt(3)*0.25 ≈ 0.43 < 0.6
    assert grid[3, 3, 3]


def test_sphere_outside_unoccupied():
    scene = Scene(name='sphere', extent=(4.0, 4.0, 4.0), resolution=0.5)
    scene.add(SceneObject(name='s', shape=Sphere(radius=0.3), position=(2.0, 2.0, 2.0)))
    grid = to_occupancy_grid(scene)
    # Voxel (0,0,0) centre (0.25,0.25,0.25) is far from (2,2,2).
    assert not grid[0, 0, 0]


def test_cylinder_centre_occupied():
    scene = Scene(name='cyl', extent=(4.0, 4.0, 4.0), resolution=0.5)
    scene.add(
        SceneObject(name='c', shape=Cylinder(radius=0.4, length=1.0), position=(2.0, 2.0, 2.0))
    )
    grid = to_occupancy_grid(scene)
    # Voxel (3,3,3) centre = (1.75, 1.75, 1.75): radial dist from (2,2) ≈ 0.354 < 0.4 ✓
    assert grid[3, 3, 3]


def test_cylinder_outside_radius_unoccupied():
    scene = Scene(name='cyl', extent=(4.0, 4.0, 4.0), resolution=0.5)
    scene.add(
        SceneObject(name='c', shape=Cylinder(radius=0.2, length=4.0), position=(2.0, 2.0, 2.0))
    )
    grid = to_occupancy_grid(scene)
    # Voxel (0,3,3) centre = (0.25, 1.75, 1.75): radial dist from (2,2) ≈ sqrt(1.75²+0.06²) >> 0.2
    assert not grid[0, 3, 3]


def test_composite_both_children_occupied():
    scene = Scene(name='comp', extent=(6.0, 4.0, 4.0), resolution=0.5)
    comp = Composite(
        children=(
            (Box(size=(1.0, 1.0, 1.0)), (-1.0, 0.0, 0.0), IDENTITY_QUAT),
            (Box(size=(1.0, 1.0, 1.0)), (1.0, 0.0, 0.0), IDENTITY_QUAT),
        )
    )
    # Composite centred at (3,2,2): child A at (2,2,2), child B at (4,2,2)
    scene.add(SceneObject(name='comp', shape=comp, position=(3.0, 2.0, 2.0)))
    grid = to_occupancy_grid(scene)
    # Voxel nearest (2,2,2): index (3,3,3), centre (1.75,1.75,1.75) — inside child A
    assert grid[3, 3, 3]
    # Voxel nearest (4,2,2): index (7,3,3), centre (3.75,1.75,1.75) — inside child B
    assert grid[7, 3, 3]


def test_room_grid_dtype_and_ndim():
    grid = to_occupancy_grid(get_scene('room'))
    assert grid.dtype == bool
    assert grid.ndim == 3


def test_room_grid_nonzero():
    grid = to_occupancy_grid(get_scene('room'))
    assert grid.any(), 'expected at least some occupied voxels in the room scene'


def test_rotated_box_dtype_shape():
    # Regression guard: non-identity rotation must not corrupt dtype or shape.
    scene = Scene(name='rot', extent=(4.0, 4.0, 4.0), resolution=0.5)
    q = euler_to_quat(0.0, 0.0, math.pi / 4)  # 45° yaw
    scene.add(
        SceneObject(
            name='b', shape=Box(size=(1.0, 1.0, 1.0)), position=(2.0, 2.0, 2.0), rotation=q
        )
    )
    grid = to_occupancy_grid(scene)
    assert grid.dtype == bool
    assert grid.shape == (8, 8, 8)
    assert grid.any()
