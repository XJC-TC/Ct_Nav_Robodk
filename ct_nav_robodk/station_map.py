"""Map ct_config arm names onto the items of a RoboDK station.

ct_config describes rails as module-frame millimetres and says nothing about how a
station models them. In NABOO-01 the three cases that exist are all different:

- ``mhr_xz`` -- ``MHR-XZ`` is a 7-axis robot whose 7th joint is the z rail (RoboDK
  reports ``z-axis rail`` as linked to it), while the x rail is an independent
  1-axis mechanism named ``x-axis rail`` sitting above it in the station tree.
- ``mhr_x`` -- ``MHR-X`` is a 7-axis robot whose 7th joint is the x rail. No z rail.
- ``mhr_u1`` / ``mhr_u2`` -- plain 6-axis robots with no rails at all.

So a rail is either a joint of the arm's own robot (``robot_axis``), a separate
mechanism to command on its own (``mechanism``), or -- for stations that model a rail
as geometry rather than kinematics -- a frame to translate (``frame``).

``scale`` / ``offset`` convert ct_config millimetres to RoboDK joint values as
``robodk = ct * scale + offset``. On NABOO-01 all rails happen to be identity: the
RoboDK joint limits match the ``travel_bounds`` in ct_config exactly (z 0-840mm,
mhr_x x 0-9365mm). Other stations may not be so lucky, hence ``calibrate_offset``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

DEFAULT_MAP_NAME = "station_map.yaml"

# A local override sits next to the tracked map and wins when present, so a
# per-machine calibration never has to be committed.
LOCAL_MAP_NAME = "station_map.local.yaml"

RAIL_KINDS = ("robot_axis", "mechanism", "frame")
FRAME_AXES = ("x", "y", "z")


class StationMapError(Exception):
    """Raised when a station map is missing, malformed, or does not fit the station."""


@dataclass(frozen=True)
class RailMap:
    """How one ct_config rail axis is driven in RoboDK."""

    axis: str
    kind: str
    joint_index: int = 1
    item: str = ""
    frame_axis: str = "x"
    scale: float = 1.0
    offset: float = 0.0

    def to_robodk(self, value_mm: float) -> float:
        return value_mm * self.scale + self.offset

    def to_ct(self, robodk_value: float) -> float:
        if self.scale == 0.0:
            raise StationMapError(f"rail {self.axis}: scale must not be zero")
        return (robodk_value - self.offset) / self.scale

    def describe(self) -> str:
        if self.kind == "robot_axis":
            where = f"robot joint {self.joint_index}"
        elif self.kind == "mechanism":
            where = f"{self.item!r} joint {self.joint_index}"
        else:
            where = f"frame {self.item!r} along {self.frame_axis}"
        return f"{self.axis} -> {where} (scale={self.scale}, offset={self.offset})"


@dataclass(frozen=True)
class ArmMap:
    name: str
    robot_item: str
    ur_joint_count: int = 6
    rails: dict[str, RailMap] = field(default_factory=dict)

    def robot_axis_rails(self) -> list[RailMap]:
        """Rails carried as extra joints of the arm's own robot, in joint order."""
        return sorted(
            (r for r in self.rails.values() if r.kind == "robot_axis"),
            key=lambda r: r.joint_index,
        )

    def external_rails(self) -> list[RailMap]:
        """Rails driven as their own item, in station-tree order (x above z)."""
        order = {"x": 0, "z": 1}
        return sorted(
            (r for r in self.rails.values() if r.kind != "robot_axis"),
            key=lambda r: order.get(r.axis, 99),
        )

    @property
    def total_joint_count(self) -> int:
        return self.ur_joint_count + len(self.robot_axis_rails())


@dataclass
class StationMap:
    station: str
    arms: dict[str, ArmMap]
    path: Path | None = None

    def arm(self, name: str) -> ArmMap:
        try:
            return self.arms[name]
        except KeyError as exc:
            raise StationMapError(
                f"No station mapping for arm {name!r} "
                f"(have: {', '.join(sorted(self.arms)) or 'none'})"
            ) from exc

    def describe(self) -> str:
        lines = [f"station: {self.station}"]
        for name in sorted(self.arms):
            arm = self.arms[name]
            lines.append(f"  {name} -> {arm.robot_item!r} ({arm.total_joint_count} joints)")
            lines.extend(f"      {rail.describe()}" for rail in arm.rails.values())
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def _parse_rail(axis: str, raw: object, where: str) -> RailMap:
    if not isinstance(raw, dict):
        raise StationMapError(f"{where}: expected a mapping")

    kind = str(raw.get("kind", "")).strip()
    if kind not in RAIL_KINDS:
        raise StationMapError(f"{where}.kind: expected one of {RAIL_KINDS}, got {kind!r}")

    item = str(raw.get("item", "")).strip()
    if kind in ("mechanism", "frame") and not item:
        raise StationMapError(f"{where}.item: required when kind is {kind!r}")

    frame_axis = str(raw.get("frame_axis", "x")).strip().lower()
    if kind == "frame" and frame_axis not in FRAME_AXES:
        raise StationMapError(f"{where}.frame_axis: expected one of {FRAME_AXES}")

    joint_index = raw.get("joint_index", 1)
    if not isinstance(joint_index, int) or joint_index < 1:
        raise StationMapError(f"{where}.joint_index: expected a 1-based integer")

    return RailMap(
        axis=axis,
        kind=kind,
        joint_index=joint_index,
        item=item,
        frame_axis=frame_axis,
        scale=float(raw.get("scale", 1.0)),
        offset=float(raw.get("offset", 0.0)),
    )


def _parse_arm(name: str, raw: object, where: str) -> ArmMap:
    if not isinstance(raw, dict):
        raise StationMapError(f"{where}: expected a mapping")

    robot_item = str(raw.get("robot_item", "")).strip()
    if not robot_item:
        raise StationMapError(f"{where}.robot_item: required")

    ur_joint_count = raw.get("ur_joint_count", 6)
    if not isinstance(ur_joint_count, int) or ur_joint_count < 1:
        raise StationMapError(f"{where}.ur_joint_count: expected a positive integer")

    rails_raw = raw.get("rails") or {}
    if not isinstance(rails_raw, dict):
        raise StationMapError(f"{where}.rails: expected a mapping of axis -> rail")

    rails = {
        axis: _parse_rail(axis, body, f"{where}.rails.{axis}")
        for axis, body in rails_raw.items()
    }
    return ArmMap(
        name=name, robot_item=robot_item, ur_joint_count=ur_joint_count, rails=rails
    )


def load_station_map(path: Path) -> StationMap:
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise StationMapError(f"Cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise StationMapError(f"Invalid YAML in {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise StationMapError(f"{path}: expected a mapping at the top level")

    arms_raw = data.get("arms")
    if not isinstance(arms_raw, dict) or not arms_raw:
        raise StationMapError(f"{path}: 'arms' is missing or empty")

    arms = {
        name: _parse_arm(name, body, f"{path.name}:arms.{name}")
        for name, body in arms_raw.items()
    }
    return StationMap(station=str(data.get("station", "")), arms=arms, path=path)


def default_map_path() -> Path:
    """The tracked map shipped with the package, or the local override if present."""
    root = Path(__file__).resolve().parent.parent
    local = root / LOCAL_MAP_NAME
    return local if local.is_file() else root / DEFAULT_MAP_NAME


def load_default_station_map() -> StationMap:
    path = default_map_path()
    if not path.is_file():
        raise StationMapError(
            f"No station map found at {path}. Run scripts/inspect_station.py and write one."
        )
    return load_station_map(path)


def save_station_map(station_map: StationMap, path: Path) -> Path:
    """Write a map back out, used by the panel's rail calibration."""
    path = Path(path)
    data = {
        "station": station_map.station,
        "arms": {
            name: {
                "robot_item": arm.robot_item,
                "ur_joint_count": arm.ur_joint_count,
                "rails": {
                    axis: {
                        "kind": rail.kind,
                        **({"item": rail.item} if rail.item else {}),
                        **({"frame_axis": rail.frame_axis} if rail.kind == "frame" else {}),
                        "joint_index": rail.joint_index,
                        "scale": rail.scale,
                        "offset": rail.offset,
                    }
                    for axis, rail in arm.rails.items()
                },
            }
            for name, arm in sorted(station_map.arms.items())
        },
    }
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, default_flow_style=False), encoding="utf-8"
    )
    return path


def with_rail_offset(
    station_map: StationMap, arm_name: str, axis: str, offset: float
) -> StationMap:
    """Copy of ``station_map`` with one rail's offset replaced."""
    arm = station_map.arm(arm_name)
    if axis not in arm.rails:
        raise StationMapError(f"{arm_name}: no {axis} rail in the station map")

    rails = dict(arm.rails)
    old = rails[axis]
    rails[axis] = RailMap(
        axis=old.axis,
        kind=old.kind,
        joint_index=old.joint_index,
        item=old.item,
        frame_axis=old.frame_axis,
        scale=old.scale,
        offset=offset,
    )
    arms = dict(station_map.arms)
    arms[arm_name] = ArmMap(
        name=arm.name,
        robot_item=arm.robot_item,
        ur_joint_count=arm.ur_joint_count,
        rails=rails,
    )
    return StationMap(station=station_map.station, arms=arms, path=station_map.path)


def calibrate_offset(rail: RailMap, robodk_value: float, ct_value_mm: float) -> float:
    """Offset that makes ``ct_value_mm`` land on the observed ``robodk_value``.

    Used when the operator jogs a rail to a position they know the ct_config value
    of; solves ``robodk = ct * scale + offset`` for the offset.
    """
    return robodk_value - ct_value_mm * rail.scale
