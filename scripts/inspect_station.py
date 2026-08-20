"""Dump the structure of the open RoboDK station.

Written to answer one question that cannot be guessed from ct_config: how each MHR
arm and its x/z rails are actually modelled in the station. On NABOO-01 the z rail
is the 7th axis of MHR-XZ's UR, the x rail is a separate item above it, and MHR-X
carries its x rail as its own 7th axis -- but item names, joint counts and joint
signs still have to be read off the real station before station_map.yaml can be
written.

RoboDK must already be running with the station open; loading a ~600 MB station
through the API is far slower than letting the GUI do it.

Usage:
    python scripts/inspect_station.py
    python scripts/inspect_station.py --filter MHR
    python scripts/inspect_station.py --json station_dump.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Running this as `python scripts/inspect_station.py` puts scripts/ on sys.path, not the
# repo root, so the project packages are not importable without this.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robodk import robolink  # noqa: E402

from ct_nav_robodk.connection import ConnectionError_, connect  # noqa: E402

# Item types worth reporting; everything else (objects, meshes, curves) is noise
# in a station with thousands of imported CAD parts.
INTERESTING_TYPES = {
    robolink.ITEM_TYPE_ROBOT,
    robolink.ITEM_TYPE_FRAME,
    robolink.ITEM_TYPE_TOOL,
    robolink.ITEM_TYPE_TARGET,
    robolink.ITEM_TYPE_PROGRAM,
}

TYPE_NAMES = {
    robolink.ITEM_TYPE_STATION: "STATION",
    robolink.ITEM_TYPE_ROBOT: "ROBOT",
    robolink.ITEM_TYPE_FRAME: "FRAME",
    robolink.ITEM_TYPE_TOOL: "TOOL",
    robolink.ITEM_TYPE_OBJECT: "OBJECT",
    robolink.ITEM_TYPE_TARGET: "TARGET",
    robolink.ITEM_TYPE_PROGRAM: "PROGRAM",
    robolink.ITEM_TYPE_MACHINING: "MACHINING",
    robolink.ITEM_TYPE_ROBOT_ARM: "ROBOT_ARM",
}


def type_name(item_type: int) -> str:
    return TYPE_NAMES.get(item_type, f"TYPE_{item_type}")


def parent_chain(item: robolink.Item) -> list[str]:
    """Names from the item up to the station root, nearest parent first."""
    chain: list[str] = []
    current = item
    for _ in range(64):  # guard against a malformed station looping on itself
        try:
            current = current.Parent()
        except Exception:
            break
        if not current.Valid():
            break
        name = current.Name()
        chain.append(name)
        if current.Type() == robolink.ITEM_TYPE_STATION:
            break
    return chain


def describe(item: robolink.Item) -> dict:
    info: dict = {
        "name": item.Name(),
        "type": type_name(item.Type()),
        "parents": parent_chain(item),
    }

    if item.Type() in (robolink.ITEM_TYPE_ROBOT, robolink.ITEM_TYPE_ROBOT_ARM):
        try:
            joints = list(item.Joints().list())
            info["dof"] = len(joints)
            info["joints"] = [round(v, 4) for v in joints]
        except Exception as exc:
            info["joints_error"] = str(exc)
        try:
            # Returns (lower, upper, joint_type) in robodk >= 5.x.
            limits = item.JointLimits()
            lower, upper = limits[0], limits[1]
            info["joint_limits"] = {
                "lower": [round(v, 3) for v in lower.list()],
                "upper": [round(v, 3) for v in upper.list()],
            }
        except Exception as exc:
            info["joint_limits_error"] = str(exc)
        try:
            link = item.getLink(robolink.ITEM_TYPE_ROBOT)
            if link.Valid() and link.Name() != item.Name():
                info["linked_robot"] = link.Name()
        except Exception:
            pass

    return info


def collect(rdk: robolink.Robolink, name_filter: str | None) -> list[dict]:
    items: list[robolink.Item] = []
    for item_type in sorted(INTERESTING_TYPES):
        items.extend(rdk.ItemList(item_type))
    # ITEM_TYPE_ROBOT does not always include single-axis mechanisms; ask explicitly.
    items.extend(rdk.ItemList(robolink.ITEM_TYPE_ROBOT_ARM))

    seen: set[str] = set()
    out: list[dict] = []
    for item in items:
        try:
            name = item.Name()
        except Exception:
            continue
        key = f"{item.Type()}:{name}"
        if key in seen:
            continue
        seen.add(key)
        if name_filter and name_filter.lower() not in name.lower():
            continue
        out.append(describe(item))
    return out


def print_report(entries: list[dict]) -> None:
    robots = [e for e in entries if e["type"] in ("ROBOT", "ROBOT_ARM")]
    others = [e for e in entries if e not in robots]

    print("=" * 78)
    print(f"Robots / mechanisms ({len(robots)})")
    print("=" * 78)
    for entry in robots:
        dof = entry.get("dof", "?")
        print(f"\n  {entry['name']}   [{entry['type']}, dof={dof}]")
        if entry.get("parents"):
            print(f"    parents: {' < '.join(entry['parents'])}")
        if "joints" in entry:
            print(f"    joints:  {entry['joints']}")
        limits = entry.get("joint_limits")
        if limits:
            print(f"    lower:   {limits['lower']}")
            print(f"    upper:   {limits['upper']}")
        if entry.get("linked_robot"):
            print(f"    linked:  {entry['linked_robot']}")
        for key in ("joints_error", "joint_limits_error"):
            if key in entry:
                print(f"    {key}: {entry[key]}")

    print()
    print("=" * 78)
    print(f"Frames / tools / targets / programs ({len(others)})")
    print("=" * 78)
    for entry in others:
        parents = " < ".join(entry.get("parents", []))
        print(f"  [{entry['type']:<8}] {entry['name']}")
        if parents:
            print(f"             parents: {parents}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--filter",
        dest="name_filter",
        help="Only report items whose name contains this substring (case-insensitive)",
    )
    parser.add_argument("--json", type=Path, help="Also write the raw dump to this JSON file")
    args = parser.parse_args()

    try:
        rdk = connect()
    except ConnectionError_ as exc:
        print(str(exc), file=sys.stderr)
        return 1

    station = rdk.ActiveStation()
    if not station.Valid():
        print("No active station in RoboDK.", file=sys.stderr)
        return 1
    print(f"Station: {station.Name()}\n")

    entries = collect(rdk, args.name_filter)
    print_report(entries)

    if args.json:
        args.json.write_text(
            json.dumps({"station": station.Name(), "items": entries}, indent=2),
            encoding="utf-8",
        )
        print(f"\nWrote {args.json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
