"""EOAT name matching, independent of a live station."""

import pytest

from ct_nav_robodk.eoat import compact_name, is_cable_guard, is_swappable_eoat_name
from robodk import robolink


@pytest.mark.parametrize(
    "name",
    [
        "CABLE GUARD",
        "Cable_Guard",
        "cable-guard",
        "MHR-XZ CABLE GUARD",
        "CableGuard",
    ],
)
def test_cable_guard_name_is_recognized(name):
    assert is_cable_guard(name)
    assert not is_swappable_eoat_name(name, robolink.ITEM_TYPE_TOOL)
    assert not is_swappable_eoat_name(name, robolink.ITEM_TYPE_OBJECT)


@pytest.mark.parametrize(
    "name, item_type",
    [
        ("Coupler EOAT", robolink.ITEM_TYPE_TOOL),
        ("Cartridge_EOAT", robolink.ITEM_TYPE_OBJECT),
        ("Syringe", robolink.ITEM_TYPE_TOOL),
        ("dcap eoat", robolink.ITEM_TYPE_FRAME),
        ("DCAP EOAT, SLEEVELESS", robolink.ITEM_TYPE_TOOL),
        ("CFC Alignment EOAT", robolink.ITEM_TYPE_TOOL),
        ("Car EOAT_op1", robolink.ITEM_TYPE_TOOL),
    ],
)
def test_swappable_eoat_names(name, item_type):
    assert is_swappable_eoat_name(name, item_type)
    assert not is_cable_guard(name)


@pytest.mark.parametrize("name", ["TCP", "Flange", "Tool", "UR10e Base"])
def test_default_tcp_names_are_not_eoats(name):
    assert not is_swappable_eoat_name(name, robolink.ITEM_TYPE_TOOL)


def test_an_unrelated_object_is_not_treated_as_an_eoat():
    assert not is_swappable_eoat_name("GSM1 shelf", robolink.ITEM_TYPE_OBJECT)


def test_compact_name_strips_punctuation():
    assert compact_name("CABLE GUARD") == compact_name("cable_guard")
