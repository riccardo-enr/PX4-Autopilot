import pytest
from px4_gz_scenes import get_scene, list_scenes, register_scene, Scene


def test_list_scenes_contains_builtins():
    scenes = list_scenes()
    assert 'room' in scenes
    assert 'office' in scenes


def test_get_scene_returns_scene():
    scene = get_scene('room')
    assert isinstance(scene, Scene)


def test_get_scene_unknown_raises():
    with pytest.raises(KeyError, match='unknown_env'):
        get_scene('unknown_env')


def test_get_scene_kwargs_forwarded():
    scene = get_scene('room', ext_x=20.0, ext_y=20.0)
    assert scene.extent[0] == 20.0
    assert scene.extent[1] == 20.0


def test_duplicate_registration_raises():
    with pytest.raises(ValueError, match='already registered'):

        @register_scene('room')
        def _duplicate(_):
            pass
