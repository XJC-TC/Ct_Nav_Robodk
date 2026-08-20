"""Station map schema and rail conversions.

Imports the submodule directly rather than the ``ct_nav_robodk`` package so the suite
still runs on a machine without the ``robodk`` package installed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from ct_nav_robodk.station_map import (
    RailMap,
    StationMapError,
    calibrate_offset,
    load_station_map,
    save_station_map,
    with_rail_offset,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
TRACKED_MAP = REPO_ROOT / "station_map.yaml"


@pytest.fixture(scope="module")
def naboo():
    return load_station_map(TRACKED_MAP)


# ---------------------------------------------------------------------------
# The shipped NABOO-01 map, checked against what inspect_station.py reported
# ---------------------------------------------------------------------------

def test_naboo_map_covers_the_four_arms(naboo):
    assert sorted(naboo.arms) == ["mhr_u1", "mhr_u2", "mhr_x", "mhr_xz"]


def test_mhr_xz_drives_z_as_joint_7_and_x_as_its_own_mechanism(naboo):
    arm = naboo.arm("mhr_xz")
    assert arm.robot_item == "MHR-XZ"
    assert arm.total_joint_count == 7

    z = arm.rails["z"]
    assert (z.kind, z.joint_index) == ("robot_axis", 7)

    x = arm.rails["x"]
    assert (x.kind, x.item, x.joint_index) == ("mechanism", "x-axis rail", 1)

    assert [r.axis for r in arm.robot_axis_rails()] == ["z"]
    assert [r.axis for r in arm.external_rails()] == ["x"]


def test_mhr_x_drives_x_as_joint_7(naboo):
    arm = naboo.arm("mhr_x")
    assert arm.total_joint_count == 7
    assert (arm.rails["x"].kind, arm.rails["x"].joint_index) == ("robot_axis", 7)
    assert "z" not in arm.rails


@pytest.mark.parametrize("name", ["mhr_u1", "mhr_u2"])
def test_railless_arms_have_six_joints(naboo, name):
    arm = naboo.arm(name)
    assert arm.rails == {}
    assert arm.total_joint_count == 6


def test_every_naboo_rail_is_identity_mapped(naboo):
    """The RoboDK joint limits already match ct_config travel_bounds on this station."""
    for arm in naboo.arms.values():
        for rail in arm.rails.values():
            assert (rail.scale, rail.offset) == (1.0, 0.0)


def test_unknown_arm_lists_the_known_ones(naboo):
    with pytest.raises(StationMapError, match="mhr_u1, mhr_u2, mhr_x, mhr_xz"):
        naboo.arm("mhr_zz")


# ---------------------------------------------------------------------------
# Conversions
# ---------------------------------------------------------------------------

def test_identity_conversion_round_trips():
    rail = RailMap(axis="x", kind="robot_axis", joint_index=7)
    assert rail.to_robodk(8240.0) == 8240.0
    assert rail.to_ct(8240.0) == 8240.0


def test_scale_and_offset_are_applied_and_inverted():
    rail = RailMap(axis="x", kind="robot_axis", joint_index=7, scale=-1.0, offset=9365.0)
    assert rail.to_robodk(0.0) == 9365.0
    assert rail.to_robodk(9365.0) == 0.0
    assert rail.to_ct(rail.to_robodk(1234.0)) == pytest.approx(1234.0)


def test_zero_scale_is_rejected_on_inverse():
    with pytest.raises(StationMapError, match="scale must not be zero"):
        RailMap(axis="x", kind="mechanism", item="r", scale=0.0).to_ct(5.0)


def test_calibrate_offset_lines_a_known_position_up():
    rail = RailMap(axis="x", kind="mechanism", item="x-axis rail")
    # Rail jogged to RoboDK 1105 while physically at the ct_config 1005mm entry node.
    offset = calibrate_offset(rail, robodk_value=1105.0, ct_value_mm=1005.0)
    assert offset == 100.0
    assert RailMap(axis="x", kind="mechanism", item="r", offset=offset).to_robodk(1005.0) == 1105.0


def test_with_rail_offset_leaves_the_original_untouched(naboo):
    updated = with_rail_offset(naboo, "mhr_xz", "x", 100.0)
    assert updated.arm("mhr_xz").rails["x"].offset == 100.0
    assert naboo.arm("mhr_xz").rails["x"].offset == 0.0
    # Other rails and arms come through unchanged.
    assert updated.arm("mhr_xz").rails["z"].offset == 0.0
    assert updated.arm("mhr_x").rails["x"] == naboo.arm("mhr_x").rails["x"]


def test_with_rail_offset_rejects_a_missing_axis(naboo):
    with pytest.raises(StationMapError, match="no z rail"):
        with_rail_offset(naboo, "mhr_x", "z", 1.0)


# ---------------------------------------------------------------------------
# Round-trip through YAML
# ---------------------------------------------------------------------------

def test_saved_map_reloads_identically(naboo, tmp_path):
    out = save_station_map(with_rail_offset(naboo, "mhr_xz", "z", -12.5), tmp_path / "m.yaml")
    reloaded = load_station_map(out)
    assert reloaded.arm("mhr_xz").rails["z"].offset == -12.5
    assert reloaded.arm("mhr_xz").rails["x"] == naboo.arm("mhr_xz").rails["x"]
    assert sorted(reloaded.arms) == sorted(naboo.arms)


# ---------------------------------------------------------------------------
# Schema errors
# ---------------------------------------------------------------------------

def write_map(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "station_map.yaml"
    path.write_text(body, encoding="utf-8")
    return path


def test_missing_arms_section_is_rejected(tmp_path):
    with pytest.raises(StationMapError, match="'arms' is missing"):
        load_station_map(write_map(tmp_path, "station: X\n"))


def test_missing_robot_item_is_rejected(tmp_path):
    body = "station: X\narms:\n  mhr_x:\n    ur_joint_count: 6\n"
    with pytest.raises(StationMapError, match="robot_item"):
        load_station_map(write_map(tmp_path, body))


def test_unknown_rail_kind_is_rejected(tmp_path):
    body = (
        "station: X\narms:\n  mhr_x:\n    robot_item: MHR-X\n"
        "    rails:\n      x:\n        kind: telekinesis\n"
    )
    with pytest.raises(StationMapError, match="kind"):
        load_station_map(write_map(tmp_path, body))


def test_mechanism_without_an_item_is_rejected(tmp_path):
    body = (
        "station: X\narms:\n  mhr_x:\n    robot_item: MHR-X\n"
        "    rails:\n      x:\n        kind: mechanism\n"
    )
    with pytest.raises(StationMapError, match="item.*required"):
        load_station_map(write_map(tmp_path, body))


def test_frame_rail_needs_a_valid_axis(tmp_path):
    body = (
        "station: X\narms:\n  mhr_x:\n    robot_item: MHR-X\n"
        "    rails:\n      x:\n        kind: frame\n        item: X_Rail\n"
        "        frame_axis: w\n"
    )
    with pytest.raises(StationMapError, match="frame_axis"):
        load_station_map(write_map(tmp_path, body))


def test_zero_based_joint_index_is_rejected(tmp_path):
    body = (
        "station: X\narms:\n  mhr_x:\n    robot_item: MHR-X\n"
        "    rails:\n      x:\n        kind: robot_axis\n        joint_index: 0\n"
    )
    with pytest.raises(StationMapError, match="1-based"):
        load_station_map(write_map(tmp_path, body))


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(StationMapError, match="Cannot read"):
        load_station_map(tmp_path / "absent.yaml")
