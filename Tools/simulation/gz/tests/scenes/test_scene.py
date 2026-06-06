from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import LABEL_FLOOR, LABEL_CEILING, LABEL_WALL, LABEL_TABLE
from px4_gz_scenes.shapes import Box
from px4_gz_scenes.scene_object import SceneObject


def test_add_boundary_creates_six_objects():
    scene = Scene(name='test', extent=(10.0, 10.0, 3.0))
    scene.add_boundary()
    assert len(scene.objects) == 6  # floor + ceiling + 4 walls


def test_boundary_labels():
    scene = Scene(name='test', extent=(10.0, 10.0, 3.0))
    scene.add_boundary()
    labels = {o.label for o in scene.objects}
    assert LABEL_FLOOR in labels
    assert LABEL_CEILING in labels
    assert LABEL_WALL in labels


def test_boundary_names():
    scene = Scene(name='test', extent=(10.0, 10.0, 3.0))
    scene.add_boundary()
    names = {o.name for o in scene.objects}
    assert names == {'floor', 'ceiling', 'wall_south', 'wall_north', 'wall_west', 'wall_east'}


def test_filter_by_label():
    scene = Scene(name='test', extent=(10.0, 10.0, 3.0))
    scene.add_boundary()
    scene.add(
        SceneObject(
            name='t1', shape=Box(size=(1.0, 1.0, 0.4)), position=(5.0, 5.0, 1.0), label=LABEL_TABLE
        )
    )
    scene.add(
        SceneObject(
            name='t2', shape=Box(size=(1.0, 1.0, 0.4)), position=(7.0, 5.0, 1.0), label=LABEL_TABLE
        )
    )
    tables = scene.filter_by_label(LABEL_TABLE)
    assert len(tables) == 2
    assert all(o.label == LABEL_TABLE for o in tables)


def test_add_floor_position():
    scene = Scene(name='test', extent=(10.0, 8.0, 3.0))
    scene.add_floor(thickness=0.2)
    floor = scene.objects[0]
    assert floor.position == (5.0, 4.0, 0.1)
    assert floor.shape.size == (10.0, 8.0, 0.2)


def test_repr():
    scene = Scene(name='test', extent=(10.0, 10.0, 3.0))
    assert 'test' in repr(scene)
    assert '0' in repr(scene)
