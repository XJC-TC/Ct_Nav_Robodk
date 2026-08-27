"""Colour-block sampling and cube meshes, no RoboDK required."""

from ct_nav_robodk.path_geom import (
    ARM_COLORS,
    BLOCK_SIZE_MM,
    CUBE_DRAW_MM,
    HIT_COLOR,
    MAX_BLOCKS_PER_ARM,
    VERTICES_PER_CUBE,
    BlockBuffer,
    LinkPoint,
    SizedCube,
    VoxelTrail,
    bone_samples,
    centers_from_voxels,
    color_for_arm,
    cube_triangles,
    cubes_vertices,
    cubes_vertices_sized,
    dedup_points,
    fill_aabb,
    fill_capsule,
    fit_cad_cubes,
    choose_local_mesh,
    is_trace_name,
    object_name,
    swept_trail_centers,
    trail_stamp_due,
    voxel_center,
    voxel_key,
    ur10e_arm_voxels,
    ur10e_link_local_voxels,
    voxels_along_chain,
    voxels_along_segment,
    voxels_at_joints,
)


def test_each_naboo_arm_has_its_own_colour():
    colours = [color_for_arm(name) for name in ("mhr_xz", "mhr_x", "mhr_u1", "mhr_u2")]
    assert len(set(colours)) == 4
    assert all(name in ARM_COLORS for name in ("mhr_xz", "mhr_x", "mhr_u1", "mhr_u2"))


def test_arm_colours_are_translucent_and_not_white_or_red():
    for name, colour in ARM_COLORS.items():
        red, green, blue, alpha = colour
        assert 0.3 <= alpha <= 0.7, name
        assert not (red > 0.9 and green > 0.9 and blue > 0.9), name
        assert not (red > 0.85 and green < 0.35 and blue < 0.35), name
    hit_r, hit_g, hit_b, _hit_a = HIT_COLOR
    assert hit_r > 0.8 and hit_g < 0.3 and hit_b < 0.3


def test_unknown_arm_falls_back_to_default():
    assert color_for_arm("mhr_zz") == color_for_arm("not-an-arm")


def test_cube_has_twelve_triangles_centred_on_the_point():
    center = (100.0, -20.0, 50.0)
    vertices = cube_triangles(center, size_mm=10.0)
    assert len(vertices) == VERTICES_PER_CUBE
    mean_x = sum(v[0] for v in vertices) / len(vertices)
    mean_y = sum(v[1] for v in vertices) / len(vertices)
    mean_z = sum(v[2] for v in vertices) / len(vertices)
    assert abs(mean_x - center[0]) < 1e-9
    assert abs(mean_y - center[1]) < 1e-9
    assert abs(mean_z - center[2]) < 1e-9
    xs = [v[0] for v in vertices]
    assert max(xs) - min(xs) == 10.0


def test_buffer_skips_a_link_that_has_not_travelled_far_enough():
    buffer = BlockBuffer(sample_mm=50.0, max_blocks=100, block_size_mm=BLOCK_SIZE_MM)
    assert buffer.consider(LinkPoint("link1", (0.0, 0.0, 0.0)))
    assert not buffer.consider(LinkPoint("link1", (10.0, 0.0, 0.0)))
    assert buffer.consider(LinkPoint("link1", (50.0, 0.0, 0.0)))
    assert buffer.count == 2


def test_buffer_tracks_each_link_independently():
    buffer = BlockBuffer(sample_mm=50.0, max_blocks=100)
    assert buffer.consider(LinkPoint("link1", (0.0, 0.0, 0.0)))
    assert buffer.consider(LinkPoint("link2", (1.0, 0.0, 0.0)))
    assert buffer.count == 2


def test_buffer_stops_at_the_cap():
    buffer = BlockBuffer(sample_mm=0.0, max_blocks=3)
    for i in range(5):
        buffer.consider(LinkPoint("link1", (float(i), 0.0, 0.0)))
    assert buffer.count == 3
    assert buffer.pending_cubes == 3


def test_take_pending_clears_the_batch():
    buffer = BlockBuffer(sample_mm=50.0, max_blocks=10)
    buffer.consider(LinkPoint("link1", (0.0, 0.0, 0.0)))
    vertices = buffer.take_pending()
    assert len(vertices) == VERTICES_PER_CUBE
    assert buffer.take_pending() == []
    assert buffer.count == 1


def test_dedup_drops_a_tcp_sitting_on_the_flange():
    flange = LinkPoint("link6", (0.0, 0.0, 0.0))
    tcp = LinkPoint("tool:EOAT", (2.0, 0.0, 0.0))
    far = LinkPoint("link1", (100.0, 0.0, 0.0))
    assert [p.key for p in dedup_points([flange, tcp, far])] == ["link6", "link1"]


def test_trace_object_names_are_stable_and_detectable():
    assert object_name("mhr_xz") == "CtNav Path mhr_xz"
    assert object_name("mhr_xz", hits=True) == "CtNav Path mhr_xz hits"
    assert object_name("mhr_xz", live="link2") == "CtNav Path mhr_xz live link2"
    assert is_trace_name("CtNav Paths")
    assert is_trace_name("CtNav Path mhr_x")
    assert is_trace_name("CtNav Path mhr_x live link0")
    assert not is_trace_name("MHR-XZ")


def test_chain_voxels_cover_every_link_not_just_the_first():
    origins = [(0.0, 0.0, 0.0), (120.0, 0.0, 0.0), (120.0, 0.0, -120.0)]
    cells = voxels_along_chain(origins, radius_mm=30.0, size_mm=30.0)
    centers = centers_from_voxels(cells, 30.0)
    xs = [c[0] for c in centers]
    zs = [c[2] for c in centers]
    assert min(xs) <= 0.0 and max(xs) >= 120.0
    assert min(zs) <= -120.0


def test_aabb_fill_covers_the_whole_box():
    cells = fill_aabb((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), size_mm=25.0)
    xs = [c[0] * 25.0 for c in cells]
    assert min(xs) <= 0.0
    assert max(xs) >= 100.0
    assert all(key[1] == 0 and key[2] == 0 for key in cells)


def test_capsule_voxels_cover_the_bone_and_its_thickness():
    cells = fill_capsule((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), radius_mm=30.0, size_mm=25.0)
    centers = centers_from_voxels(cells, 25.0)
    xs = [c[0] for c in centers]
    assert min(xs) <= 0.0
    assert max(xs) >= 100.0
    assert any(abs(c[1]) >= 25.0 for c in centers)


def test_voxel_trail_paints_each_cell_once():
    trail = VoxelTrail(size_mm=25.0, max_blocks=10)
    first = trail.absorb([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
    assert len(first) == 1
    assert trail.absorb([(1.0, 0.0, 0.0)]) == []
    second = trail.absorb([(50.0, 0.0, 0.0)])
    assert len(second) == 1
    assert trail.count == 2


def test_cubes_vertices_is_one_cube_per_centre():
    verts = cubes_vertices([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)], size_mm=10.0)
    assert len(verts) == VERTICES_PER_CUBE * 2


def test_bone_samples_fill_the_gap_between_consecutive_links():
    joints = [
        LinkPoint("link1", (0.0, 0.0, 0.0)),
        LinkPoint("link2", (120.0, 0.0, 0.0)),
    ]
    filled = bone_samples(joints, step_mm=40.0)
    keys = [p.key for p in filled]
    assert "link1" in keys and "link2" in keys
    mids = [p for p in filled if p.key.startswith("link1-link2@")]
    assert mids
    assert abs(mids[0].xyz[0] - 40.0) < 1e-6


def test_would_add_matches_consider_without_mutating():
    buffer = BlockBuffer(sample_mm=50.0, max_blocks=10)
    point = LinkPoint("link1", (0.0, 0.0, 0.0))
    assert buffer.would_add(point)
    assert buffer.count == 0
    assert buffer.consider(point)
    assert not buffer.would_add(LinkPoint("link1", (10.0, 0.0, 0.0)))


def test_joint_voxels_cover_housings_not_the_whole_tube():
    origins = [(0.0, 0.0, 0.0), (400.0, 0.0, 0.0)]
    cells = voxels_at_joints(origins, radius_mm=64.0, size_mm=32.0)
    assert voxel_key((0.0, 0.0, 0.0), 32.0) in cells
    assert voxel_key((400.0, 0.0, 0.0), 32.0) in cells
    assert voxel_key((200.0, 0.0, 0.0), 32.0) not in cells


def test_ur10e_model_wraps_the_tube_and_the_joints():
    origins = [(0.0, 0.0, 0.0), (612.0, 0.0, 0.0), (612.0, 0.0, -572.0)]
    cells = ur10e_arm_voxels(origins, size_mm=30.0)
    assert voxel_key((0.0, 0.0, 0.0), 30.0) in cells
    assert voxel_key((612.0, 0.0, 0.0), 30.0) in cells
    assert voxel_key((306.0, 0.0, 0.0), 30.0) in cells
    assert voxel_key((612.0, 0.0, -286.0), 30.0) in cells


def test_ur10e_model_skips_a_rail_length_bone():
    origins = [(0.0, 0.0, 0.0), (2000.0, 0.0, 0.0)]
    cells = ur10e_arm_voxels(origins, size_mm=30.0)
    assert voxel_key((0.0, 0.0, 0.0), 30.0) in cells
    assert voxel_key((1000.0, 0.0, 0.0), 30.0) not in cells


def test_ur10e_local_link_runs_along_the_next_joint():
    cells = ur10e_link_local_voxels((612.0, 0.0, 0.0), index=2, last_index=6, size_mm=40.0)
    assert voxel_key((0.0, 0.0, 0.0), 40.0) in cells
    assert voxel_key((300.0, 0.0, 0.0), 40.0) in cells
    assert voxel_key((612.0, 0.0, 0.0), 40.0) in cells


def test_drawn_cubes_on_the_grid_do_not_overlap():
    left = voxel_center((0, 0, 0), BLOCK_SIZE_MM)
    right = voxel_center((1, 0, 0), BLOCK_SIZE_MM)
    half = CUBE_DRAW_MM / 2.0
    assert (right[0] - half) - (left[0] + half) > 0.0


def test_trail_defaults_are_coarse_enough_for_a_long_run():
    assert BLOCK_SIZE_MM == 80.0
    assert MAX_BLOCKS_PER_ARM == 12000
    trail = VoxelTrail()
    assert trail.size_mm == 80.0
    assert trail.max_blocks == 12000


def test_fit_cad_does_not_fill_an_l_notch():
    bar_x = [(float(x), 0.0, 0.0) for x in range(0, 81, 10)]
    bar_y = [(0.0, float(y), 0.0) for y in range(10, 81, 10)]
    cubes = fit_cad_cubes(bar_x + bar_y)
    assert cubes
    assert not any(
        abs(cube.center[0] - 80.0) < 30.0 and abs(cube.center[1] - 80.0) < 30.0
        for cube in cubes
    )


def test_fit_cad_rejects_a_rail_length_cloud():
    points = [(0.0, 0.0, 0.0), (2000.0, 0.0, 0.0), (1000.0, 10.0, 0.0)]
    assert fit_cad_cubes(points) == []


def test_fit_cad_merges_a_solid_block_to_large_cubes():
    points = [
        (float(x), float(y), float(z))
        for x in range(-40, 41, 10)
        for y in range(-40, 41, 10)
        for z in range(-40, 41, 10)
    ]
    cubes = fit_cad_cubes(points)
    assert cubes
    assert any(cube.size_mm == 80.0 for cube in cubes)


def test_swept_trail_fills_the_gap_between_poses():
    centers = swept_trail_centers([(0.0, 0.0, 0.0)], [(240.0, 0.0, 0.0)], size_mm=80.0)
    xs = [c[0] for c in centers]
    assert min(xs) <= 0.0
    assert max(xs) >= 240.0
    assert any(abs(x - 80.0) < 1e-6 for x in xs)
    assert any(abs(x - 160.0) < 1e-6 for x in xs)


def test_voxels_along_segment_includes_both_ends():
    cells = voxels_along_segment((0.0, 0.0, 0.0), (80.0, 0.0, 0.0), size_mm=80.0)
    assert voxel_key((0.0, 0.0, 0.0), 80.0) in cells
    assert voxel_key((80.0, 0.0, 0.0), 80.0) in cells


def test_sized_cube_vertices_scale_with_the_cube():
    cubes = [SizedCube((0.0, 0.0, 0.0), 20.0), SizedCube((100.0, 0.0, 0.0), 80.0)]
    verts = cubes_vertices_sized(cubes, draw_scale=1.0)
    assert len(verts) == VERTICES_PER_CUBE * 2
    small = verts[:VERTICES_PER_CUBE]
    large = verts[VERTICES_PER_CUBE:]
    assert max(v[0] for v in small) - min(v[0] for v in small) == 20.0
    assert max(v[0] for v in large) - min(v[0] for v in large) == 80.0


def test_choose_local_mesh_prefers_the_cloud_near_the_origin():
    raw = [(2000.0, 100.0, 50.0), (2020.0, 100.0, 50.0), (2010.0, 120.0, 50.0)]
    converted = [(10.0, 0.0, 0.0), (30.0, 0.0, 0.0), (20.0, 20.0, 0.0)]
    assert choose_local_mesh(raw, converted) == converted


def test_choose_local_mesh_keeps_an_offset_local_cad():
    raw = [(200.0, 0.0, 40.0), (220.0, 0.0, 40.0), (210.0, 20.0, 40.0)]
    converted = [(4000.0, 800.0, 600.0), (4020.0, 800.0, 600.0), (4010.0, 820.0, 600.0)]
    assert choose_local_mesh(raw, converted) == raw


def test_trail_stamp_due_on_force_or_first_pose():
    current = [(0.0, 0.0, 0.0)]
    assert trail_stamp_due(None, current, ticks=1, force=False)
    assert trail_stamp_due(current, current, ticks=1, force=True)
    assert not trail_stamp_due(current, current, ticks=1, force=False)


def test_trail_stamp_due_after_n_ticks_or_enough_travel():
    start = [(0.0, 0.0, 0.0)]
    near = [(10.0, 0.0, 0.0)]
    far = [(80.0, 0.0, 0.0)]
    assert not trail_stamp_due(start, near, ticks=2, force=False, every_n=3, move_mm=80.0)
    assert trail_stamp_due(start, near, ticks=3, force=False, every_n=3, move_mm=80.0)
    assert trail_stamp_due(start, far, ticks=1, force=False, every_n=3, move_mm=80.0)
