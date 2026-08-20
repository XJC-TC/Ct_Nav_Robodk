import pytest

from ct_nav.park_poses import (
    PARK_POSES,
    is_park_name,
    normalize_park_name,
    park_joints,
)


def test_both_locations_define_the_same_four_poses():
    assert set(PARK_POSES["lower"]) == set(PARK_POSES["upper"])
    assert len(PARK_POSES["lower"]) == 4


@pytest.mark.parametrize(
    "raw, expected",
    [
        ("ORTHOGONAL_PARK", "ORTHOGONAL_PARK"),
        ("'ORTHOGONAL_PARK'", "ORTHOGONAL_PARK"),
        ('"PARALLEL_UP_PARK"', "PARALLEL_UP_PARK"),
        ("  parallel_down_park  ", "PARALLEL_DOWN_PARK"),
        ("Wrist Up Park", "WRIST_UP_PINS_DOWN"),
        ("WRIST_UP_PINS_DOWN", "WRIST_UP_PINS_DOWN"),
    ],
)
def test_normalize_accepts_the_quoting_ct_config_uses(raw, expected):
    assert normalize_park_name(raw) == expected


@pytest.mark.parametrize("raw", ["enter_0", "pick_place_node", "", None, 5])
def test_non_park_names_normalize_to_empty(raw):
    assert normalize_park_name(raw) == ""
    assert not is_park_name(raw)


def test_park_joints_uses_the_caller_base_and_the_location_table():
    lower = park_joints("ORTHOGONAL_PARK", "lower", -90.0)
    assert lower == [-90.0, -70.6, -110.1, -89.3, 90.0, 0.0]

    upper = park_joints("ORTHOGONAL_PARK", "upper", 90.0)
    assert upper == [90.0, -33.71, -144.31, -91.98, -90.0, 0.0]


def test_park_joints_always_returns_six_joints():
    for location in ("lower", "upper"):
        for name in PARK_POSES[location]:
            assert len(park_joints(name, location, 0.0)) == 6


def test_unknown_park_or_location_raises():
    with pytest.raises(KeyError):
        park_joints("SOME_NODE", "lower", 0.0)
    with pytest.raises(KeyError):
        park_joints("ORTHOGONAL_PARK", "sideways", 0.0)
