"""
SDF exporter — convert a Scene to a Gazebo world file.

Each SceneObject becomes a static ``<model>`` in the world.  The world
boilerplate (physics, lighting, spherical coordinates) matches the TU Delft
CyberZoo location so that PX4's GPS simulation behaves consistently.

Coordinate convention: the Scene uses ENU (x=East, y=North, z=Up), which is
the same orientation used by Gazebo when the world declares
``<world_frame_orientation>ENU</world_frame_orientation>``.  No axis
remapping is needed.

Supported shapes: Box, Cylinder, Sphere, Composite.
For Composite shapes each child becomes a separate collision + visual pair
inside a single link.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from px4_gz_scenes._types import Quaternion, Vec3
from px4_gz_scenes.scene import Scene
from px4_gz_scenes.scene_object import (
    LABEL_CEILING,
    LABEL_COLUMN,
    LABEL_FLOOR,
    LABEL_PARTITION,
    LABEL_RACK,
    LABEL_TABLE,
    LABEL_WALL,
    SceneObject,
)
from px4_gz_scenes.shapes import Box, Composite, Cylinder, Shape, Sphere

# ── Colour palette (ambient/diffuse RGB) per semantic label ────────────────

_LABEL_COLOR: dict[str, tuple[float, float, float]] = {
    LABEL_FLOOR: (0.75, 0.75, 0.75),
    LABEL_CEILING: (0.90, 0.90, 0.90),
    LABEL_WALL: (0.80, 0.80, 0.80),
    LABEL_PARTITION: (0.65, 0.65, 0.70),
    LABEL_TABLE: (0.55, 0.38, 0.18),
    LABEL_COLUMN: (0.45, 0.45, 0.45),
    LABEL_RACK: (0.25, 0.28, 0.30),
}
_DEFAULT_COLOR: tuple[float, float, float] = (0.60, 0.60, 0.60)


# ── Internal helpers ───────────────────────────────────────────────────────


def _quat_to_rpy(q: Quaternion) -> tuple[float, float, float]:
    """Convert a Hamilton quaternion (w, x, y, z) to ZYX Euler angles (rad)."""
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _fmt(v: float) -> str:
    """Format a float, stripping trailing zeros after the decimal point."""
    s = f"{v:.6g}"
    return s


def _pose_str(position: Vec3, rotation: Quaternion) -> str:
    """Return a Gazebo pose string ``'x y z roll pitch yaw'``."""
    x, y, z = position
    roll, pitch, yaw = _quat_to_rpy(rotation)
    # Round position to 6 decimal places to suppress float subtraction noise.
    return f"{round(x, 6)} {round(y, 6)} {round(z, 6)} {roll:.6f} {pitch:.6f} {yaw:.6f}"


def _geometry_xml(shape: Shape, indent: str) -> str:
    """Return the ``<geometry>...</geometry>`` block for *shape*."""
    i = indent
    if isinstance(shape, Box):
        sx, sy, sz = (round(v, 6) for v in shape.size)
        return f"{i}<geometry>\n{i}  <box><size>{sx} {sy} {sz}</size></box>\n{i}</geometry>\n"
    if isinstance(shape, Cylinder):
        return (
            f"{i}<geometry>\n"
            f"{i}  <cylinder>"
            f"<radius>{shape.radius}</radius>"
            f"<length>{shape.length}</length>"
            f"</cylinder>\n"
            f"{i}</geometry>\n"
        )
    if isinstance(shape, Sphere):
        return (
            f"{i}<geometry>\n"
            f"{i}  <sphere><radius>{shape.radius}</radius></sphere>\n"
            f"{i}</geometry>\n"
        )
    raise TypeError(f"Unsupported shape type for SDF export: {type(shape)}")


def _material_xml(color: tuple[float, float, float], indent: str) -> str:
    r, g, b = color
    i = indent
    return (
        f"{i}<material>\n"
        f"{i}  <ambient>{r} {g} {b} 1</ambient>\n"
        f"{i}  <diffuse>{r} {g} {b} 1</diffuse>\n"
        f"{i}  <specular>0.1 0.1 0.1 1</specular>\n"
        f"{i}</material>\n"
    )


def _collision_visual_pair(
    shape: Shape,
    color: tuple[float, float, float],
    offset: Vec3 = (0.0, 0.0, 0.0),
    rotation: Quaternion = (1.0, 0.0, 0.0, 0.0),
    index: int = 0,
    indent: str = "          ",
) -> str:
    """Return collision + visual XML for one shape, optionally offset within the link."""
    i = indent
    pose = _pose_str(offset, rotation)
    geom = _geometry_xml(shape, i + "  ")
    mat = _material_xml(color, i + "  ")
    tag = "" if index == 0 else f"_{index}"
    return (
        f'{i}<collision name="collision{tag}">\n'
        f"{i}  <pose>{pose}</pose>\n"
        f"{geom}"
        f"{i}</collision>\n"
        f'{i}<visual name="visual{tag}">\n'
        f"{i}  <pose>{pose}</pose>\n"
        f"{geom}"
        f"{mat}"
        f"{i}</visual>\n"
    )


def _model_xml(obj: SceneObject) -> str:
    """Return the full ``<model>...</model>`` block for *obj*."""
    color = _LABEL_COLOR.get(obj.label, _DEFAULT_COLOR)
    i4 = "    "
    i8 = "        "

    if isinstance(obj.shape, Composite):
        # Each child is a (shape, offset, rotation) triple relative to obj.position.
        pairs = "".join(
            _collision_visual_pair(
                child_shape,
                color,
                offset=child_offset,
                rotation=child_rot,
                index=k,
            )
            for k, (child_shape, child_offset, child_rot) in enumerate(
                obj.shape.children
            )
        )
    else:
        pairs = _collision_visual_pair(obj.shape, color)

    pose = _pose_str(obj.position, obj.rotation)
    return (
        f'{i4}<model name="{obj.name}">\n'
        f"{i4}  <static>true</static>\n"
        f"{i4}  <pose>{pose}</pose>\n"
        f'{i4}  <link name="link">\n'
        f"{pairs}"
        f"{i4}  </link>\n"
        f"{i4}</model>\n"
    )


# ── World boilerplate ──────────────────────────────────────────────────────

_WORLD_HEADER = """\
<?xml version="1.0" encoding="UTF-8" ?>
<sdf version="1.9">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic" />
    <scene>
      <grid>false</grid>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.5 0.5 0.5 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>51.9906361</latitude_deg>
      <longitude_deg>4.3767874</longitude_deg>
      <elevation>45.110</elevation>
    </spherical_coordinates>
"""

_WORLD_FOOTER = """\
  </world>
</sdf>
"""


# ── Public API ─────────────────────────────────────────────────────────────


def scene_to_sdf(
    scene: Scene,
    exclude_labels: Sequence[str] | None = None,
) -> str:
    """Convert *scene* to a complete Gazebo SDF world string.

    Args:
        scene:          The scene to export.
        exclude_labels: Optional list of label strings to omit from the output
                        (e.g. ``[LABEL_CEILING]`` to keep the interior visible
                        during development).

    Returns:
        A UTF-8 SDF string ready to write to a ``.sdf`` file.
    """
    skip = set(exclude_labels) if exclude_labels else set()
    models = "".join(_model_xml(obj) for obj in scene.objects if obj.label not in skip)
    header = _WORLD_HEADER.format(world_name=scene.name)
    return header + models + _WORLD_FOOTER
