"""Cube-vs-entity overlap, independent of RoboDK."""

from ct_nav_robodk.collision import (
    CollisionHit,
    CollisionReport,
    CubeOccupancy,
    EntityIndex,
    aabb_from_points,
    bone_aabb,
    cube_aabb,
    first_overlap,
    keep_entity_name,
    on_own_body,
)
from ct_nav_robodk.path_geom import BLOCK_SIZE_MM, is_trace_name


def test_cubes_overlap_when_they_share_volume():
    a = cube_aabb((0.0, 0.0, 0.0), 20.0)
    b = cube_aabb((10.0, 0.0, 0.0), 20.0)
    c = cube_aabb((50.0, 0.0, 0.0), 20.0)
    assert a.overlaps(b)
    assert not a.overlaps(c)


def test_first_overlap_returns_the_named_entity():
    cube = cube_aabb((0.0, 0.0, 0.0), 20.0, name="link3")
    wall = cube_aabb((5.0, 0.0, 0.0), 20.0, name="IOM1")
    far = cube_aabb((200.0, 0.0, 0.0), 20.0, name="GSM")
    hit = first_overlap(cube, [far, wall])
    assert hit is not None
    assert hit.name == "IOM1"


def test_bone_aabb_covers_the_segment_plus_radius():
    box = bone_aabb((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), radius_mm=10.0, name="MHR-X")
    assert box.minimum[0] == -10.0
    assert box.maximum[0] == 110.0
    assert cube_aabb((50.0, 0.0, 0.0), 10.0).overlaps(box)


def test_aabb_from_points_is_the_axis_aligned_hull():
    box = aabb_from_points([(1.0, 2.0, 3.0), (4.0, -1.0, 8.0)])
    assert box is not None
    assert box.minimum == (1.0, -1.0, 3.0)
    assert box.maximum == (4.0, 2.0, 8.0)


def test_trace_and_hidden_names_are_not_indexed():
    assert not keep_entity_name("CtNav Path mhr_xz", visible=True)
    assert not keep_entity_name("IOM1", visible=False)
    assert keep_entity_name("IOM1", visible=True)
    assert is_trace_name("CtNav Paths")


def test_occupancy_ignores_the_same_arm_and_hits_another():
    grid = CubeOccupancy(cell_mm=BLOCK_SIZE_MM)
    grid.add("mhr_xz", (0.0, 0.0, 0.0))
    cube = cube_aabb((0.0, 0.0, 0.0), BLOCK_SIZE_MM, name="link1")
    assert grid.hit(cube, ignore_arm="mhr_xz") is None
    grid.add("mhr_x", (5.0, 0.0, 0.0))
    assert grid.hit(cube, ignore_arm="mhr_xz") == "mhr_x path cube"


def test_index_reports_static_then_other_cubes():
    index = EntityIndex()
    index.static = [cube_aabb((100.0, 0.0, 0.0), 40.0, name="IOM1")]
    cube = cube_aabb((100.0, 0.0, 0.0), 28.0, name="link2")
    hit = index.hit(cube, ignore_arm="mhr_xz")
    assert hit == CollisionHit(cube_key="link2", entity="IOM1")
    assert CollisionReport(hits=(hit,)).describe() == "cube link2 vs IOM1"


def test_empty_report_is_not_a_hit():
    report = CollisionReport()
    assert not report.hit
    assert report.describe() == "no collision"


def test_cube_on_the_arm_envelope_is_own_body():
    own = [bone_aabb((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), radius_mm=50.0, name="self")]
    assert on_own_body(cube_aabb((50.0, 0.0, 0.0), 22.0, name="link2"), own)
    assert not on_own_body(cube_aabb((400.0, 0.0, 0.0), 22.0, name="tcp"), own)


def test_index_skips_entities_already_touching_at_start():
    index = EntityIndex()
    index.static = [cube_aabb((0.0, 0.0, 0.0), 80.0, name="MHR-XZ")]
    cube = cube_aabb((0.0, 0.0, 0.0), 22.0, name="link0")
    assert index.hit(cube, ignore_arm="mhr_xz") == CollisionHit(
        cube_key="link0", entity="MHR-XZ"
    )
    assert index.hit(cube, ignore_arm="mhr_xz", skip_entities={"MHR-XZ"}) is None
    wall = cube_aabb((200.0, 0.0, 0.0), 40.0, name="IOM1")
    index.static.append(wall)
    far = cube_aabb((200.0, 0.0, 0.0), 22.0, name="tcp")
    assert index.hit(far, ignore_arm="mhr_xz", skip_entities={"MHR-XZ"}) == CollisionHit(
        cube_key="tcp", entity="IOM1"
    )
