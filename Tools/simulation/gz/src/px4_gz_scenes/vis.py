"""
3-D visualisation for px4_gz_scenes environments.

Renders a :class:`~px4_gz_scenes.scene.Scene` interactively using Matplotlib's
3-D axes. Each SceneObject is drawn as a solid cuboid (boxes), cylinder, or
sphere, colour-coded by semantic label.

Usage::

    from px4_gz_scenes import get_scene
    from px4_gz_scenes.vis import visualise_scene

    visualise_scene(get_scene("room"))
    visualise_scene(get_scene("office"), show_labels=True)

Requires ``matplotlib`` (``pip install matplotlib`` or ``pixi add matplotlib``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from px4_gz_scenes.scene import Scene
    from px4_gz_scenes.scene_object import SceneObject

# ── Label → colour map ──────────────────────────────────────────────────────
_LABEL_COLORS: dict[str, str] = {
    'floor': '#c8c8c8',  # light grey
    'ceiling': '#e0e0e0',  # very light grey
    'wall': '#9e9e9e',  # mid grey
    'partition': '#5c8ed4',  # blue
    'table': '#8d6439',  # brown
    'column': '#c0392b',  # red
    'rack': '#e67e22',  # orange
    'door': '#27ae60',  # green
    'obstacle': '#7f8c8d',  # grey-blue (default)
}
_DEFAULT_COLOR = '#7f8c8d'
_DEFAULT_ALPHA = 0.35


def _label_color(label: str) -> str:
    return _LABEL_COLORS.get(label, _DEFAULT_COLOR)


# ── Cuboid drawing ───────────────────────────────────────────────────────────


def _cuboid_vertices(
    cx: float,
    cy: float,
    cz: float,
    sx: float,
    sy: float,
    sz: float,
) -> np.ndarray:
    """Return the 8 vertices of an axis-aligned cuboid centred at (cx,cy,cz)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    return np.array(
        [
            [cx - hx, cy - hy, cz - hz],
            [cx + hx, cy - hy, cz - hz],
            [cx + hx, cy + hy, cz - hz],
            [cx - hx, cy + hy, cz - hz],
            [cx - hx, cy - hy, cz + hz],
            [cx + hx, cy - hy, cz + hz],
            [cx + hx, cy + hy, cz + hz],
            [cx - hx, cy + hy, cz + hz],
        ]
    )


_CUBOID_FACES = [
    [0, 1, 2, 3],  # bottom
    [4, 5, 6, 7],  # top
    [0, 1, 5, 4],  # front
    [2, 3, 7, 6],  # back
    [1, 2, 6, 5],  # right
    [0, 3, 7, 4],  # left
]


def _draw_box(ax, cx, cy, cz, sx, sy, sz, color, alpha):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    verts = _cuboid_vertices(cx, cy, cz, sx, sy, sz)
    faces = [[verts[i] for i in face] for face in _CUBOID_FACES]
    poly = Poly3DCollection(faces, alpha=alpha, linewidths=0.3, edgecolors='k')
    poly.set_facecolor(color)
    ax.add_collection3d(poly)


def _draw_cylinder(ax, cx, cy, cz, radius, length, color, alpha, n=24):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    xs = radius * np.cos(theta)
    ys = radius * np.sin(theta)
    z0 = cz - length / 2
    z1 = cz + length / 2

    # Side faces
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append(
            [
                [cx + xs[i], cy + ys[i], z0],
                [cx + xs[j], cy + ys[j], z0],
                [cx + xs[j], cy + ys[j], z1],
                [cx + xs[i], cy + ys[i], z1],
            ]
        )
    # Cap faces
    bottom = [[cx + xs[i], cy + ys[i], z0] for i in range(n)]
    top = [[cx + xs[i], cy + ys[i], z1] for i in range(n)]
    faces.append(bottom)
    faces.append(top)

    poly = Poly3DCollection(faces, alpha=alpha, linewidths=0.2, edgecolors='k')
    poly.set_facecolor(color)
    ax.add_collection3d(poly)


def _draw_sphere(ax, cx, cy, cz, radius, color, alpha, n=16):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    phi = np.linspace(0, np.pi, n)
    theta = np.linspace(0, 2 * np.pi, n)
    faces = []
    for i in range(n - 1):
        for j in range(n - 1):
            p00 = [
                cx + radius * np.sin(phi[i]) * np.cos(theta[j]),
                cy + radius * np.sin(phi[i]) * np.sin(theta[j]),
                cz + radius * np.cos(phi[i]),
            ]
            p10 = [
                cx + radius * np.sin(phi[i + 1]) * np.cos(theta[j]),
                cy + radius * np.sin(phi[i + 1]) * np.sin(theta[j]),
                cz + radius * np.cos(phi[i + 1]),
            ]
            p11 = [
                cx + radius * np.sin(phi[i + 1]) * np.cos(theta[j + 1]),
                cy + radius * np.sin(phi[i + 1]) * np.sin(theta[j + 1]),
                cz + radius * np.cos(phi[i + 1]),
            ]
            p01 = [
                cx + radius * np.sin(phi[i]) * np.cos(theta[j + 1]),
                cy + radius * np.sin(phi[i]) * np.sin(theta[j + 1]),
                cz + radius * np.cos(phi[i]),
            ]
            faces.append([p00, p10, p11, p01])
    poly = Poly3DCollection(faces, alpha=alpha, linewidths=0.0)
    poly.set_facecolor(color)
    ax.add_collection3d(poly)


def _draw_object(ax, obj: SceneObject) -> None:
    from px4_gz_scenes.shapes import Box, Cylinder, Sphere, Composite

    color = _label_color(obj.label)
    alpha = 0.15 if obj.label in ('floor', 'ceiling', 'wall') else _DEFAULT_ALPHA
    cx, cy, cz = obj.position

    shape = obj.shape
    if isinstance(shape, Box):
        _draw_box(ax, cx, cy, cz, *shape.size, color, alpha)

    elif isinstance(shape, Cylinder):
        _draw_cylinder(ax, cx, cy, cz, shape.radius, shape.length, color, alpha)

    elif isinstance(shape, Sphere):
        _draw_sphere(ax, cx, cy, cz, shape.radius, color, alpha)

    elif isinstance(shape, Composite):
        for child_shape, (ox, oy, oz), _rot in shape.children:
            child_obj_pos = (cx + ox, cy + oy, cz + oz)
            from px4_gz_scenes.scene_object import SceneObject as SO

            _draw_object(
                ax,
                SO(
                    name=obj.name,
                    shape=child_shape,
                    position=child_obj_pos,
                    label=obj.label,
                ),
            )


# ── Legend ───────────────────────────────────────────────────────────────────


def _add_legend(ax, labels: set[str]) -> None:
    import matplotlib.patches as mpatches

    handles = [
        mpatches.Patch(facecolor=_label_color(l), edgecolor='k', linewidth=0.5, label=l)
        for l in sorted(labels)
    ]
    ax.legend(handles=handles, loc='upper left', fontsize=7, framealpha=0.7)


# ── Public API ───────────────────────────────────────────────────────────────


def visualise_scene(
    scene: Scene,
    *,
    show_labels: bool = True,
    elev: float = 25.0,
    azim: float = -60.0,
    figsize: tuple[float, float] = (9.0, 7.0),
    title: str | None = None,
) -> None:
    """Render *scene* interactively in a Matplotlib 3-D window.

    Args:
        scene:        The scene to render.
        show_labels:  If True, add a colour legend for semantic labels.
        elev:         Initial elevation angle of the camera (degrees).
        azim:         Initial azimuth angle of the camera (degrees).
        figsize:      Figure size in inches ``(width, height)``.
        title:        Window / axes title. Defaults to ``scene.name``.
    """
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3D projection
    except ImportError as exc:
        raise ImportError(
            'Visualisation requires matplotlib. Install it with: pip install matplotlib'
        ) from exc

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    labels_used: set[str] = set()
    for obj in scene.objects:
        _draw_object(ax, obj)
        labels_used.add(obj.label)

    if show_labels:
        _add_legend(ax, labels_used)

    ex, ey, ez = scene.extent
    ax.set_xlim(0, ex)
    ax.set_ylim(0, ey)
    ax.set_zlim(0, ez)
    ax.set_xlabel('X / East (m)')
    ax.set_ylabel('Y / North (m)')
    ax.set_zlabel('Z / Up (m)')
    ax.set_title(title or scene.name)
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect([ex, ey, ez])

    plt.tight_layout()
    plt.show()
