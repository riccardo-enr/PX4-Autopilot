from px4_gz_scenes.shapes import Box, Cylinder, Sphere, Composite, aabb
from px4_gz_scenes._types import IDENTITY_QUAT


def test_box_aabb():
    lo, hi = aabb(Box(size=(2.0, 4.0, 6.0)))
    assert lo == (-1.0, -2.0, -3.0)
    assert hi == (1.0, 2.0, 3.0)


def test_cylinder_aabb():
    lo, hi = aabb(Cylinder(radius=1.5, length=4.0))
    assert lo == (-1.5, -1.5, -2.0)
    assert hi == (1.5, 1.5, 2.0)


def test_sphere_aabb():
    lo, hi = aabb(Sphere(radius=2.0))
    assert lo == (-2.0, -2.0, -2.0)
    assert hi == (2.0, 2.0, 2.0)


def test_composite_aabb():
    box_a = Box(size=(2.0, 2.0, 2.0))  # centred at (0,0,0), AABB: ±1
    box_b = Box(size=(2.0, 2.0, 2.0))  # offset by (4,0,0) → AABB: [3,5]×[-1,1]×[-1,1]
    comp = Composite(
        children=(
            (box_a, (0.0, 0.0, 0.0), IDENTITY_QUAT),
            (box_b, (4.0, 0.0, 0.0), IDENTITY_QUAT),
        )
    )
    lo, hi = aabb(comp)
    assert lo == (-1.0, -1.0, -1.0)
    assert hi == (5.0, 1.0, 1.0)
