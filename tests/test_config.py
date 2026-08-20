"""Config loading, both against synthetic YAML and the real azula1 checkout."""

from __future__ import annotations

import pytest

from ct_nav import RailPose, load_arm
from ct_nav.config import ConfigError, load_cluster
from ct_nav.park_poses import is_park_name

MINIMAL_CLUSTER_CONFIG = """
highway_tree:
  root:
    parent: null
    rail_pose:
      x: 0 mm
  far:
    parent: root
    rail_pose:
      x: 500 mm
arm_nav_trees:
  mod1:
    thing:
      rail_pose:
        x: 510 mm
      arm_tree_pointer: some_tree.thing
      highway_node: far
module_edges: {}
"""

MINIMAL_TREE = """
trees:
  thing:
    enter_0:
      parent: 'ORTHOGONAL_PARK'
      pose:
        base: 1.0 deg
        shoulder: 2.0 deg
        elbow: 3.0 deg
        wrist_1: 4.0 deg
        wrist_2: 5.0 deg
        wrist_3: 6.0 deg
    pick_place_node:
      parent: enter_0
      pose:
        base: 11.0 deg
        shoulder: 12.0 deg
        elbow: 13.0 deg
        wrist_1: 14.0 deg
        wrist_2: 15.0 deg
        wrist_3: 16.0 deg
"""


def write_arm(root, *, cluster_config=MINIMAL_CLUSTER_CONFIG, tree=MINIMAL_TREE, rails=True):
    arm = root / "mhr_fake"
    (arm / "navigation").mkdir(parents=True)
    (arm / "cluster_config.yaml").write_text(cluster_config, encoding="utf-8")
    (arm / "navigation" / "some_tree.yaml").write_text(tree, encoding="utf-8")
    (arm / "ur12e.yaml").write_text("location: lower\npark_base_angle: -90 deg\n", encoding="utf-8")
    if rails:
        (arm / "x_rail.yaml").write_text(
            "drive_attributes:\n  travel_bounds:\n    upper: 600 mm\n    lower: 0 mm\n",
            encoding="utf-8",
        )
    return arm


# ---------------------------------------------------------------------------
# Synthetic
# ---------------------------------------------------------------------------

def test_loads_a_minimal_arm(tmp_path):
    arm = load_arm(write_arm(tmp_path))

    assert arm.name == "mhr_fake"
    assert arm.location == "lower"
    assert arm.park_base_angle_deg == -90.0
    assert arm.robot_model == "ur12e"
    assert arm.highway_root() == "root"
    assert arm.rail_axes() == ("x",)
    assert arm.rail_limits["x"].upper_mm == 600.0

    target = arm.nav_target("mod1", "thing")
    assert target.tree_file == "some_tree"
    assert target.tree_name == "thing"
    assert target.highway_node == "far"
    assert target.rail_pose == RailPose(x=510.0)

    tree = arm.tree(target)
    assert tree.nodes["enter_0"].joints == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def test_dangling_highway_parent_is_rejected(tmp_path):
    bad = MINIMAL_CLUSTER_CONFIG.replace("parent: root", "parent: nowhere")
    with pytest.raises(ConfigError, match="not a highway node"):
        load_arm(write_arm(tmp_path, cluster_config=bad))


def test_missing_joint_in_a_pose_is_rejected(tmp_path):
    bad = MINIMAL_TREE.replace("        wrist_3: 6.0 deg\n", "")
    with pytest.raises(ConfigError, match="missing"):
        load_arm(write_arm(tmp_path, tree=bad))


def test_bad_arm_tree_pointer_is_rejected(tmp_path):
    bad = MINIMAL_CLUSTER_CONFIG.replace("some_tree.thing", "no_dot_here")
    with pytest.raises(ConfigError, match="arm_tree_pointer"):
        load_arm(write_arm(tmp_path, cluster_config=bad))


def test_missing_tree_is_reported_on_lookup(tmp_path):
    arm = load_arm(write_arm(tmp_path, tree="trees: {}\n"))
    with pytest.raises(ConfigError, match="missing tree"):
        arm.tree(arm.nav_target("mod1", "thing"))


def test_cluster_needs_at_least_one_arm(tmp_path):
    (tmp_path / "not_an_arm").mkdir()
    with pytest.raises(ConfigError, match="no arm directories"):
        load_cluster(tmp_path)


# ---------------------------------------------------------------------------
# RailPose
# ---------------------------------------------------------------------------

def test_rail_pose_axes_are_in_station_tree_order():
    assert list(RailPose(x=1.0, z=2.0).axes()) == ["x", "z"]


def test_rail_pose_merge_keeps_axes_the_other_pose_omits():
    state = RailPose(x=100.0, z=18.0)
    assert state.merged_with(RailPose(x=200.0)) == RailPose(x=200.0, z=18.0)
    assert state.merged_with(RailPose()) == state


def test_empty_rail_pose():
    assert RailPose().is_empty()
    assert RailPose().describe() == "-"
    assert not RailPose(x=0.0).is_empty()


def test_unknown_rail_axis_is_rejected():
    with pytest.raises(ConfigError, match="unknown rail axes"):
        RailPose.from_yaml({"y": "1 mm"}, "where")


# ---------------------------------------------------------------------------
# Real azula1 checkout
# ---------------------------------------------------------------------------

def test_azula1_has_the_four_expected_arms(cluster):
    assert cluster.arm_names() == ["mhr_u1", "mhr_u2", "mhr_x", "mhr_xz"]


@pytest.mark.parametrize(
    "arm_name, location, rails",
    [
        ("mhr_xz", "lower", ("x", "z")),
        ("mhr_x", "lower", ("x",)),
        ("mhr_u1", "upper", ()),
        ("mhr_u2", "upper", ()),
    ],
)
def test_azula1_arm_shapes(cluster, arm_name, location, rails):
    arm = cluster.arm(arm_name)
    assert arm.location == location
    assert arm.rail_axes() == rails


def test_azula1_mhr_xz_highway_matches_the_yaml(cluster):
    arm = cluster.arm("mhr_xz")
    assert arm.highway_root() == "iom_entry"
    assert arm.highway["gsm1_2"].rail_pose == RailPose(x=8240.0, z=18.0)
    assert arm.highway["sim_exit"].parent == "sim_entry"


def test_azula1_mhr_u1_has_only_the_dummy_highway_node(cluster):
    arm = cluster.arm("mhr_u1")
    assert list(arm.highway) == ["dummy"]
    assert arm.highway["dummy"].rail_pose.is_empty()


def test_every_azula1_nav_target_resolves_to_a_real_tree(cluster):
    for arm in cluster.arms.values():
        for module in arm.modules():
            for target in arm.targets(module):
                nav_target = arm.nav_target(module, target)
                tree = arm.tree(nav_target)
                assert tree.nodes, f"{arm.name} {nav_target.label} points at an empty tree"
                assert nav_target.highway_node in arm.highway


def test_every_azula1_tree_root_hangs_off_a_known_park_pose(cluster):
    for arm in cluster.arms.values():
        for key, tree in arm.trees.items():
            for root in tree.roots():
                assert is_park_name(root.parent or ""), (
                    f"{arm.name} {key}.{root.name} has non-park root parent {root.parent!r}"
                )


def test_tuck_away_prefers_tucked_away_not_the_enter_approach(cluster):
    for arm_name in cluster.arm_names():
        arm = cluster.arm(arm_name)
        if "tuck_away" not in {t for m in arm.modules() for t in arm.targets(m)}:
            continue
        for module in arm.modules():
            if "tuck_away" not in arm.targets(module):
                continue
            tree = arm.tree(arm.nav_target(module, "tuck_away"))
            assert list(tree.nodes)[0] == "enter"
            assert tree.preferred_node() == "tucked_away"
            assert [n.name for n in tree.leaves()] == ["tucked_away"]


def test_azula1_rail_limits_are_loaded(cluster):
    xz = cluster.arm("mhr_xz")
    assert xz.rail_limits["x"].upper_mm == 8800.0
    assert xz.rail_limits["z"].upper_mm == 840.0
    # mhr_x's x rail reaches further than mhr_xz's, which is why the station's
    # x-axis rail mechanism tops out at 9365mm rather than 8800mm.
    assert cluster.arm("mhr_x").rail_limits["x"].upper_mm == 9365.0
