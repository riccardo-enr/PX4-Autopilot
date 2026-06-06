import pytest
from px4_gz_scenes import get_scene
from px4_gz_scenes.scene_object import LABEL_TABLE, LABEL_PARTITION, LABEL_COLUMN, LABEL_RACK


@pytest.mark.parametrize('name', ['room', 'office'])
def test_scene_has_objects(name):
    scene = get_scene(name)
    assert len(scene.objects) > 0


@pytest.mark.parametrize('name', ['room', 'office'])
def test_all_objects_within_extent(name):
    """Every object's centre must lie within [0, extent] on each axis."""
    scene = get_scene(name)
    ex, ey, ez = scene.extent
    for obj in scene.objects:
        px, py, pz = obj.position
        assert 0.0 <= px <= ex, f'{obj.name} x={px} outside [0, {ex}]'
        assert 0.0 <= py <= ey, f'{obj.name} y={py} outside [0, {ey}]'
        assert 0.0 <= pz <= ez, f'{obj.name} z={pz} outside [0, {ez}]'


def test_room_has_expected_labels():
    scene = get_scene('room')
    labels = {o.label for o in scene.objects}
    assert LABEL_TABLE in labels
    assert LABEL_PARTITION in labels
    assert LABEL_COLUMN in labels


def test_office_has_expected_labels():
    scene = get_scene('office')
    labels = {o.label for o in scene.objects}
    assert LABEL_TABLE in labels
    assert LABEL_PARTITION in labels
    assert LABEL_COLUMN in labels
    assert LABEL_RACK in labels


def test_room_custom_size():
    scene = get_scene('room', ext_x=20.0, ext_y=20.0, z_height=4.0)
    assert scene.extent == (20.0, 20.0, 4.0)
    # All objects must still be within the larger extent.
    for obj in scene.objects:
        assert obj.position[0] <= 20.0
        assert obj.position[1] <= 20.0
        assert obj.position[2] <= 4.0


def test_office_object_count():
    scene = get_scene('office')
    # 6 boundary + 5 partitions + 4 desks + 2 columns + 1 rack = 18
    assert len(scene.objects) == 18


def test_room_object_count():
    scene = get_scene('room')
    # 6 boundary + 2 half-walls + 2 tables + 1 column = 11
    assert len(scene.objects) == 11
