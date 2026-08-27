"""Time our lightweight cube/AABB path check on the open station.

Does not call RoboDK Collisions(). Requires RoboDK with a station loaded.

Usage:
    python scripts/spike_path_collision.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ct_nav_robodk.connection import connect  # noqa: E402
from ct_nav_robodk.driver import resolve_arm  # noqa: E402
from ct_nav_robodk.path_geom import color_for_arm  # noqa: E402
from ct_nav_robodk.path_trace import (  # noqa: E402
    build_static_index,
    collect_link_points,
    other_robot_bones,
)
from ct_nav_robodk.station_map import load_default_station_map  # noqa: E402


def _ms(started: float) -> float:
    return (time.perf_counter() - started) * 1000.0


def main() -> int:
    rdk = connect()
    station_map = load_default_station_map()
    print(f"station map: {station_map.station}")

    first_arm = next(iter(station_map.arms))
    items = resolve_arm(rdk, station_map.arm(first_arm))

    started = time.perf_counter()
    static = build_static_index(rdk, items.robot)
    print(f"static AABB index: {_ms(started):.1f} ms  n={len(static)}")

    started = time.perf_counter()
    bones = other_robot_bones(rdk, items.robot)
    print(f"other-robot bones: {_ms(started):.1f} ms  n={len(bones)}")

    for arm_name in station_map.arms:
        arm_items = resolve_arm(rdk, station_map.arm(arm_name))
        started = time.perf_counter()
        points = collect_link_points(arm_items)
        print(
            f"  {arm_name} ({color_for_arm(arm_name)[:3]}): "
            f"{len(points)} body samples in {_ms(started):.1f} ms"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
