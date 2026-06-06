import numpy as np
import pytest
from px4_gz_scenes import get_scene, list_scenes
from px4_gz_scenes.pointcloud import sample_pointcloud
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import SceneObject
from px4_gz_scenes.shapes import Box


ALL_SCENES = list_scenes()


# -- Fixtures ----------------------------------------------------------------


@pytest.fixture
def small_box_scene():
    """Single 1x1x1 box centred at (2,2,2) inside a 4x4x4 extent."""
    scene = Scene(name='small_box', extent=(4.0, 4.0, 4.0))
    scene.add(SceneObject(name='box', shape=Box(size=(1.0, 1.0, 1.0)), position=(2.0, 2.0, 2.0)))
    return scene


# -- Tests -------------------------------------------------------------------


def test_output_shape_and_dtype(small_box_scene):
    pc = sample_pointcloud(small_box_scene, 50.0, rng=np.random.default_rng(0))
    assert pc.ndim == 2
    assert pc.shape[1] == 3
    assert pc.dtype == np.float32


def test_nonempty(small_box_scene):
    pc = sample_pointcloud(small_box_scene, 50.0, rng=np.random.default_rng(0))
    assert pc.shape[0] > 0


@pytest.mark.parametrize('scene_name', ALL_SCENES)
def test_points_within_extent(scene_name):
    scene = get_scene(scene_name)
    pc = sample_pointcloud(scene, 20.0, rng=np.random.default_rng(42))
    ex, ey, ez = scene.extent
    eps = 0.01
    assert pc[:, 0].min() >= -eps
    assert pc[:, 1].min() >= -eps
    assert pc[:, 2].min() >= -eps
    assert pc[:, 0].max() <= ex + eps
    assert pc[:, 1].max() <= ey + eps
    assert pc[:, 2].max() <= ez + eps


@pytest.mark.parametrize('scene_name', ALL_SCENES)
def test_reproducible_with_rng(scene_name):
    scene = get_scene(scene_name)
    pc1 = sample_pointcloud(scene, 20.0, rng=np.random.default_rng(0))
    pc2 = sample_pointcloud(scene, 20.0, rng=np.random.default_rng(0))
    np.testing.assert_array_equal(pc1, pc2)


@pytest.mark.parametrize('scene_name', ALL_SCENES)
def test_nonempty_registered_scene(scene_name):
    pc = sample_pointcloud(get_scene(scene_name), 20.0, rng=np.random.default_rng(0))
    assert pc.shape[0] > 0


def test_density_proportional():
    """A larger box should produce more points than a smaller one at the same density."""
    big = Scene(name='big', extent=(10.0, 10.0, 10.0))
    big.add(SceneObject(name='b', shape=Box(size=(4.0, 4.0, 4.0)), position=(5.0, 5.0, 5.0)))

    small = Scene(name='small', extent=(10.0, 10.0, 10.0))
    small.add(SceneObject(name='s', shape=Box(size=(1.0, 1.0, 1.0)), position=(5.0, 5.0, 5.0)))

    pc_big = sample_pointcloud(big, 50.0, rng=np.random.default_rng(0))
    pc_small = sample_pointcloud(small, 50.0, rng=np.random.default_rng(0))
    assert pc_big.shape[0] > pc_small.shape[0]
