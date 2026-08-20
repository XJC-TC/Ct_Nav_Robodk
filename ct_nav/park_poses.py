"""The PARK poses that ct_config navigation trees hang off but never define.

Every root node in a ``navigation/*.yaml`` tree has a ``parent`` naming a park pose
(``ORTHOGONAL_PARK``, ``PARALLEL_UP_PARK``, ``PARALLEL_DOWN_PARK``,
``WRIST_UP_PINS_DOWN``) instead of another node. Those joint values live in the MHR
software, not in ct_config, so they are reproduced here from the reference table in
``non_prod_tool/Nodes_transfer_app/index_v2.html`` (``REFERENCE_POSES``), which is
the tool the navigation nodes are authored with.

Which of the two tables applies depends on the arm's ``location`` (``lower`` /
``upper``) from its ``ur10e.yaml`` / ``ur12e.yaml``.

``base`` is deliberately absent: a park pose is a shape, not a full pose.
``park_joints`` takes the base angle from the caller. Navigation and Reset to PARK
both pass the arm's ``park_base_angle`` from ``ur*.yaml`` (typically +90 or -90).
"""

from __future__ import annotations

# Joint order after base: shoulder, elbow, wrist_1, wrist_2, wrist_3.
_LOWER = {
    "ORTHOGONAL_PARK": (-70.6, -110.1, -89.3, 90.0, 0.0),
    "PARALLEL_UP_PARK": (-30.0, -120.0, -210.0, -90.0, 0.0),
    "PARALLEL_DOWN_PARK": (-55.0, -125.0, 180.0, -90.0, 180.0),
    "WRIST_UP_PINS_DOWN": (-60.0, -120.0, 0.0, 90.0, 0.0),
}

_UPPER = {
    "ORTHOGONAL_PARK": (-33.71, -144.31, -91.98, -90.0, 0.0),
    "PARALLEL_DOWN_PARK": (-28.49, -139.26, -12.21, 90.0, 180.0),
    "PARALLEL_UP_PARK": (-28.5, -139.26, -12.22, 90.0, 0.0),
    "WRIST_UP_PINS_DOWN": (-35.87, -118.79, -205.31, -90.0, 0.0),
}

PARK_POSES = {"lower": _LOWER, "upper": _UPPER}

# The tool accepts WRIST_UP_PARK as an alias for the same pose.
_ALIASES = {"WRIST_UP_PARK": "WRIST_UP_PINS_DOWN"}

PARK_NAMES = frozenset(_LOWER) | frozenset(_ALIASES)


def normalize_park_name(name: str) -> str:
    """Canonical park name, or ``""`` when ``name`` is not a park pose at all.

    Tree ``parent`` values are quoted inconsistently in ct_config, so whitespace and
    case are normalized before matching.
    """
    if not isinstance(name, str):
        return ""
    key = name.strip().strip("'\"").upper().replace(" ", "_")
    key = _ALIASES.get(key, key)
    return key if key in _LOWER else ""


def is_park_name(name: str) -> bool:
    return bool(normalize_park_name(name))


def park_joints(name: str, location: str, base_deg: float) -> list[float]:
    """Six joint angles in degrees for ``name`` at ``location``, using ``base_deg``."""
    canonical = normalize_park_name(name)
    if not canonical:
        raise KeyError(f"Not a park pose: {name!r}")
    table = PARK_POSES.get(location.strip().lower())
    if table is None:
        raise KeyError(f"Unknown arm location {location!r} (expected 'lower' or 'upper')")
    return [float(base_deg), *table[canonical]]
