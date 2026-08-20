"""Parse the unit-suffixed scalars used throughout ct_config YAML.

ct_config writes every physical quantity as a string with a unit suffix
(``"8240 mm"``, ``"-90.0 deg"``, ``"200 mm/s^2"``), and YAML therefore loads them
as ``str`` rather than numbers. A few entries are plain numbers (``x: 0``), so
both forms have to be accepted.
"""

from __future__ import annotations

import re

_SCALAR_RE = re.compile(
    r"""^\s*
    (?P<value>[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)
    \s*
    (?P<unit>[A-Za-z][A-Za-z0-9/^*]*)?
    \s*$""",
    re.VERBOSE,
)

# Everything ct_config uses, normalized to mm and deg.
_LENGTH_TO_MM = {"mm": 1.0, "cm": 10.0, "m": 1000.0}
_ANGLE_TO_DEG = {"deg": 1.0, "degree": 1.0, "degrees": 1.0, "rad": 57.29577951308232}


class UnitError(ValueError):
    """Raised when a ct_config scalar cannot be parsed or has the wrong dimension."""


def _split(raw: object, field: str) -> tuple[float, str | None]:
    if isinstance(raw, bool):
        raise UnitError(f"{field}: expected a scalar, got a bool")
    if isinstance(raw, (int, float)):
        return float(raw), None
    if not isinstance(raw, str):
        raise UnitError(f"{field}: expected a number or string, got {type(raw).__name__}")

    match = _SCALAR_RE.match(raw)
    if not match:
        raise UnitError(f"{field}: cannot parse {raw!r}")
    return float(match.group("value")), match.group("unit")


def parse_mm(raw: object, field: str = "value") -> float:
    """Return a length in millimetres. A bare number is assumed to be mm already."""
    value, unit = _split(raw, field)
    if unit is None:
        return value
    factor = _LENGTH_TO_MM.get(unit.lower())
    if factor is None:
        raise UnitError(f"{field}: {raw!r} is not a length")
    return value * factor


def parse_deg(raw: object, field: str = "value") -> float:
    """Return an angle in degrees. A bare number is assumed to be degrees already."""
    value, unit = _split(raw, field)
    if unit is None:
        return value
    factor = _ANGLE_TO_DEG.get(unit.lower())
    if factor is None:
        raise UnitError(f"{field}: {raw!r} is not an angle")
    return value * factor
