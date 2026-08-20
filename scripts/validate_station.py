"""Drive navigation nodes in the open station and check the station ends up where ct_config says.

This is the end-to-end check that the station map is right. For each case it plans a
move, applies it, then reads the station back and compares:

- every rail, against the ``rail_pose`` the plan targeted (in ct_config millimetres);
- the six arm joints, against the ``pose`` in the tree YAML for the selected node.

A mismatch means the station map's ``joint_index`` / ``scale`` / ``offset`` is wrong for
that rail, or the arm's joint order does not match ct_config's.

RoboDK must be running with the station open. The arms are moved, so run it on a station
you have not got unsaved work in.

Usage:
    python scripts/validate_station.py
    python scripts/validate_station.py --cluster D:\\Bitbucket\\ct_config\\azula1
    python scripts/validate_station.py --all-arms --animate
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ct_nav import ArmConfig, ClusterConfig, Mode, load_cluster, plan_move  # noqa: E402
from ct_nav_robodk.connection import ConnectionError_, active_station_name, connect  # noqa: E402
from ct_nav_robodk.driver import (  # noqa: E402
    Driver,
    DriverError,
    DriverOptions,
    read_rail_pose,
)
from ct_nav_robodk.station_map import StationMapError, load_default_station_map  # noqa: E402

DEFAULT_CLUSTER = Path(r"D:\Bitbucket\ct_config\azula1")

# Chosen to exercise every distinct shape in azula1: mhr_xz crossing between modules on
# both rails, an mhr_xz target whose highway hop is a single step, mhr_x with only an x
# rail carried as the robot's 7th axis, and a railless mhr_u.
CASES: tuple[tuple[str, str, str, str], ...] = (
    ("mhr_xz", "iom1", "hotel_e_bag_cartridge", "pick_place_node"),
    ("mhr_xz", "icm_grex1", "drawer_0", "drawer_opened"),
    ("mhr_xz", "gsm2", "bag_b_cartridge", "pick_place_node"),
    ("mhr_xz", "ncm1", "nc_cassette", "nc200"),
    ("mhr_x", "gsm1", "coupler", "syringe_a"),
    ("mhr_u1", "mhr1", "tuck_away", "tucked_away"),
    ("mhr_u2", "mhr1", "tuck_away", "tucked_away"),
)

RAIL_TOLERANCE_MM = 0.01
JOINT_TOLERANCE_DEG = 0.01


@dataclass
class Result:
    label: str
    passed: bool
    detail: str


def _expected_rail(plan) -> dict[str, float]:
    """The rail position the plan intends to finish at, per axis."""
    expected: dict[str, float] = dict(plan.start_rail.axes())
    for step in plan.steps:
        expected.update(step.rail.axes())
    return expected


def check_case(
    driver: Driver,
    arm: ArmConfig,
    module: str,
    target: str,
    node: str,
    mode: Mode,
) -> Result:
    label = f"{arm.name} {module}.{target}.{node}"
    try:
        plan = plan_move(arm, module, target, node, mode=mode)
    except Exception as exc:
        return Result(label, False, f"planning failed: {exc}")

    try:
        items = driver.items_for(arm.name)
        driver.run(plan)
    except (DriverError, StationMapError) as exc:
        return Result(label, False, f"driving failed: {exc}")

    problems: list[str] = []

    expected_rail = _expected_rail(plan)
    actual_rail = read_rail_pose(items)
    for axis, wanted in expected_rail.items():
        got = getattr(actual_rail, axis, None)
        if got is None:
            problems.append(f"{axis} rail is not mapped in the station map")
        elif abs(got - wanted) > RAIL_TOLERANCE_MM:
            problems.append(f"{axis} rail is {got:.3f}mm, cluster_config says {wanted:.3f}mm")

    tree = arm.tree(arm.nav_target(module, target))
    wanted_joints = tree.nodes[node].joints
    got_joints = items.robot.Joints().list()[: items.arm_map.ur_joint_count]
    for index, (wanted, got) in enumerate(zip(wanted_joints, got_joints)):
        if abs(got - wanted) > JOINT_TOLERANCE_DEG:
            problems.append(f"joint {index + 1} is {got:.3f}deg, tree YAML says {wanted:.3f}deg")

    if problems:
        return Result(label, False, "; ".join(problems))

    rails = actual_rail.describe() if expected_rail else "no rails"
    return Result(label, True, f"{len(plan.steps)} steps, rails {rails}")


def cases_for(cluster: ClusterConfig, all_arms: bool) -> list[tuple[str, str, str, str]]:
    available = [case for case in CASES if case[0] in cluster.arms]
    if not all_arms:
        return available

    # One target per arm/module pair, so an unfamiliar cluster still gets broad coverage.
    extra: list[tuple[str, str, str, str]] = []
    for arm in cluster.arms.values():
        for module in arm.modules():
            for target in arm.targets(module):
                nav_target = arm.nav_target(module, target)
                tree = arm.tree(nav_target)
                if not tree.nodes:
                    continue
                extra.append((arm.name, module, target, list(tree.nodes)[-1]))
                break
    seen: set[tuple[str, str, str, str]] = set()
    return [case for case in available + extra if not (case in seen or seen.add(case))]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--cluster", type=Path, default=DEFAULT_CLUSTER)
    parser.add_argument(
        "--mode",
        type=Mode,
        default=Mode.FULL,
        choices=list(Mode),
        help="Which planning mode to validate (default: full)",
    )
    parser.add_argument(
        "--all-arms",
        action="store_true",
        help="Also check one target per arm/module pair, not just the curated cases",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Interpolate the motion so it can be watched (much slower)",
    )
    parser.add_argument("--speed", type=float, default=8.0, help="Animation speed multiplier")
    args = parser.parse_args()

    try:
        cluster = load_cluster(args.cluster)
    except Exception as exc:
        print(f"Cannot load {args.cluster}: {exc}", file=sys.stderr)
        return 1

    try:
        rdk = connect()
        station_map = load_default_station_map()
    except (ConnectionError_, StationMapError) as exc:
        print(str(exc), file=sys.stderr)
        return 1

    driver = Driver(
        rdk, station_map, DriverOptions(animate=args.animate, speed=args.speed)
    )
    problems = driver.verify(name for name in cluster.arm_names() if name in station_map.arms)
    if problems:
        print("Station map does not fit this station:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    print(f"Station: {active_station_name(rdk)}")
    print(f"Cluster: {cluster.path}  ({args.mode.label})\n")

    results = [
        check_case(driver, cluster.arm(arm_name), module, target, node, args.mode)
        for arm_name, module, target, node in cases_for(cluster, args.all_arms)
    ]

    for result in results:
        print(f"  {'PASS' if result.passed else 'FAIL'}  {result.label}")
        print(f"        {result.detail}")

    failed = [r for r in results if not r.passed]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
