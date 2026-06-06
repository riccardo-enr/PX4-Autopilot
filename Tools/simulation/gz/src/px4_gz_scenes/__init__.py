"""
px4_gz_scenes — declarative Python scene definitions for PX4/Gazebo research.

Public API
----------
Shapes::

    Box, Cylinder, Sphere, Composite, Shape, aabb

Scene objects::

    SceneObject
    LABEL_WALL, LABEL_FLOOR, LABEL_CEILING, LABEL_TABLE,
    LABEL_COLUMN, LABEL_PARTITION, LABEL_RACK, LABEL_DOOR, LABEL_OBSTACLE

Scene container::

    Scene

Registry::

    get_scene, register_scene, list_scenes

Types / utilities::

    Vec3, Quaternion, Color, IDENTITY_QUAT, euler_to_quat

Visualisation (requires matplotlib)::

    visualise_scene

Exporters::

    to_occupancy_grid
    sample_pointcloud
    scene_to_sdf

Typical usage::

    from px4_gz_scenes import get_scene, list_scenes, visualise_scene

    print(list_scenes())          # ['office', 'room']
    scene = get_scene("room")
    scene = get_scene("office", ext_x=20.0)

    for obj in scene.filter_by_label("table"):
        print(obj.name, obj.position)

    visualise_scene(scene)
"""

from px4_gz_scenes._types import (
    Color,
    IDENTITY_QUAT,
    Quaternion,
    Vec3,
    euler_to_quat,
)
from px4_gz_scenes.registry import get_scene, list_scenes, register_scene
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import (
    LABEL_CEILING,
    LABEL_COLUMN,
    LABEL_DOOR,
    LABEL_FLOOR,
    LABEL_OBSTACLE,
    LABEL_PARTITION,
    LABEL_RACK,
    LABEL_TABLE,
    LABEL_WALL,
    SceneObject,
)
from px4_gz_scenes.shapes import Box, Composite, Cylinder, Shape, Sphere, aabb
from px4_gz_scenes.vis import visualise_scene
from px4_gz_scenes.occupancy import to_occupancy_grid
from px4_gz_scenes.pointcloud import sample_pointcloud
from px4_gz_scenes.sdf import scene_to_sdf

# Importing environments triggers @register_scene decorators.
import px4_gz_scenes.environments  # noqa: F401

__all__ = [
    # types
    'Vec3',
    'Quaternion',
    'Color',
    'IDENTITY_QUAT',
    'euler_to_quat',
    # shapes
    'Box',
    'Cylinder',
    'Sphere',
    'Composite',
    'Shape',
    'aabb',
    # scene object
    'SceneObject',
    'LABEL_WALL',
    'LABEL_FLOOR',
    'LABEL_CEILING',
    'LABEL_TABLE',
    'LABEL_COLUMN',
    'LABEL_PARTITION',
    'LABEL_RACK',
    'LABEL_DOOR',
    'LABEL_OBSTACLE',
    # scene
    'Scene',
    # registry
    'get_scene',
    'register_scene',
    'list_scenes',
    # visualisation
    'visualise_scene',
    # exporters
    'sample_pointcloud',
    'to_occupancy_grid',
    'scene_to_sdf',
]
