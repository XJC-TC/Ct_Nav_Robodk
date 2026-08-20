"""Turn a ``Plan`` into RoboDK targets and programs.

The live driver is for looking; this is for keeping. An exported program survives the
panel closing and can be fed to RoboDK's collision checking and cycle-time estimation,
which need real instructions rather than ``setJoints`` calls.

Consumes the same ``Plan.steps`` as the driver so a previewed move and an exported
program cannot describe different motion.

One RoboDK limitation shapes the output: a program's instructions all belong to a
single robot, so a rail modelled as an independent mechanism (``mhr_xz``'s x rail)
cannot be interleaved into the arm's program. Those arms get a companion program for
the rail plus comment instructions in the main program recording the value the rail
should hold at each step, so the sequence stays auditable. Rails carried as an extra
joint of the arm itself (``mhr_xz``'s z, ``mhr_x``'s x) need none of this -- they are
part of every joint target.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from robodk import robolink

from ct_nav.planner import Plan

from .driver import ArmItems, resolve_arm
from .station_map import StationMap

DEFAULT_PREFIX = "CtNav"

# RoboDK item names are matched by string, so anything that reads as a path separator or
# breaks the name lookup has to go. Tree names like "drawer_0/1" make this necessary.
_UNSAFE_NAME = re.compile(r"[\\/:*?\"<>|]+")


class ExportError(Exception):
    """Raised when a plan cannot be turned into a program."""


@dataclass
class ExportResult:
    frame: robolink.Item
    program: robolink.Item
    targets: list[robolink.Item] = field(default_factory=list)
    rail_programs: dict[str, robolink.Item] = field(default_factory=dict)
    rail_targets: dict[str, list[robolink.Item]] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def summary(self) -> str:
        parts = [f"{self.program.Name()}: {len(self.targets)} targets"]
        for axis, program in sorted(self.rail_programs.items()):
            parts.append(f"{program.Name()}: {len(self.rail_targets.get(axis, []))} targets")
        return "; ".join(parts)


def safe_name(text: str) -> str:
    return _UNSAFE_NAME.sub("-", text).strip()


def program_name(plan: Plan, prefix: str = DEFAULT_PREFIX) -> str:
    return safe_name(
        f"{prefix} {plan.arm} {plan.nav_target.label}.{plan.node} "
        f"[{plan.mode.value}/{plan.visit.value}]"
    )


def _replace_item(rdk: robolink.Robolink, name: str, item_type: int) -> None:
    """Delete an existing item of this name so a re-export overwrites rather than piles up."""
    item = rdk.Item(name, item_type)
    if item.Valid():
        item.Delete()


def _get_or_add_frame(rdk: robolink.Robolink, name: str) -> robolink.Item:
    frame = rdk.Item(name, robolink.ITEM_TYPE_FRAME)
    if frame.Valid():
        return frame
    return rdk.AddFrame(name)


def _add_joint_target(
    rdk: robolink.Robolink,
    name: str,
    frame: robolink.Item,
    robot: robolink.Item,
    joints: list[float],
) -> robolink.Item:
    target = rdk.AddTarget(safe_name(name), frame, robot)
    if not target.Valid():
        raise ExportError(f"RoboDK refused to create target {name!r}")
    target.setAsJointTarget()
    target.setJoints(joints)
    return target


def _joint_baselines(items: ArmItems, plan: Plan) -> tuple[list[float], dict[str, float]]:
    """Values to use for joints a step does not command.

    A joint target has to pin every joint, so a park step that only moves the arm still
    needs a number for the z rail. Preference order is the plan's first commanded value,
    then the rail position the plan assumed it was starting from, and only then the live
    station. That keeps the exported program self-contained: replaying it from anywhere
    reproduces the same motion instead of baking export-time state into target 1.

    The middle case is not hypothetical. ``mhr_x``'s only highway node sits at the same
    x as most of its targets, so a plan starting there emits no rail move at all and the
    steps alone never mention x.
    """
    arm_baseline = list(items.robot.Joints().list())[: items.arm_map.ur_joint_count]
    for step in plan.steps:
        if step.joints is not None:
            arm_baseline = [float(v) for v in step.joints]
            break

    rail_baseline: dict[str, float] = {}
    for rail_map in items.arm_map.robot_axis_rails():
        commanded = next(
            (
                value
                for step in plan.steps
                if (value := getattr(step.rail, rail_map.axis, None)) is not None
            ),
            None,
        )
        if commanded is None:
            commanded = getattr(plan.start_rail, rail_map.axis, None)
        if commanded is not None:
            rail_baseline[rail_map.axis] = commanded
    return arm_baseline, rail_baseline


def _full_joint_vector(
    items: ArmItems,
    plan: Plan,
    step_index: int,
    arm_baseline: list[float],
    rail_baseline: dict[str, float],
) -> list[float]:
    """Arm joints plus robot-axis rails for one step, carrying values forward.

    A step that only moves rails keeps the previous step's arm pose and vice versa, so
    every target is absolute.
    """
    arm_map = items.arm_map
    joints = list(items.robot.Joints().list())

    arm_pose = list(arm_baseline)
    rail_values = dict(rail_baseline)
    for step in plan.steps[: step_index + 1]:
        if step.joints is not None:
            arm_pose = [float(v) for v in step.joints]
        for axis, value in step.rail.axes().items():
            rail_values[axis] = value

    joints[: arm_map.ur_joint_count] = arm_pose
    for rail_map in arm_map.robot_axis_rails():
        if rail_map.axis in rail_values:
            joints[rail_map.joint_index - 1] = rail_map.to_robodk(rail_values[rail_map.axis])
    return joints


def export_plan(
    rdk: robolink.Robolink,
    station_map: StationMap,
    plan: Plan,
    prefix: str = DEFAULT_PREFIX,
) -> ExportResult:
    """Create targets and a program reproducing ``plan``, replacing any earlier export."""
    if not plan.steps:
        raise ExportError("Nothing to export: the plan has no steps")

    items = resolve_arm(rdk, station_map.arm(plan.arm))
    base_name = program_name(plan, prefix)

    _replace_item(rdk, base_name, robolink.ITEM_TYPE_PROGRAM)
    frame = _get_or_add_frame(rdk, safe_name(f"{prefix} {plan.arm} Targets"))

    program = rdk.AddProgram(base_name, items.robot)
    if not program.Valid():
        raise ExportError(f"RoboDK refused to create program {base_name!r}")
    program.setFrame(frame)

    external = items.arm_map.external_rails()
    rail_programs: dict[str, robolink.Item] = {}
    rail_frames: dict[str, robolink.Item] = {}
    for rail_map in external:
        rail_name = safe_name(f"{base_name} [{rail_map.axis}-rail]")
        _replace_item(rdk, rail_name, robolink.ITEM_TYPE_PROGRAM)
        rail_item = items.rail_items[rail_map.axis]
        rail_program = rdk.AddProgram(rail_name, rail_item)
        if not rail_program.Valid():
            raise ExportError(f"RoboDK refused to create program {rail_name!r}")
        rail_programs[rail_map.axis] = rail_program
        rail_frames[rail_map.axis] = frame

    targets: list[robolink.Item] = []
    rail_targets: dict[str, list[robolink.Item]] = {axis: [] for axis in rail_programs}
    notes: list[str] = []

    program.RunInstruction(
        f"ct_config {plan.arm} {plan.nav_target.label}.{plan.node} "
        f"({plan.mode.label}, {plan.visit.label})",
        robolink.INSTRUCTION_COMMENT,
    )
    if plan.highway_route:
        program.RunInstruction(
            "highway: " + " -> ".join(plan.highway_route), robolink.INSTRUCTION_COMMENT
        )

    # Track external rail values so consecutive steps holding the same value emit one
    # move rather than several. Deliberately starts empty rather than reading the
    # station: an exported program has to be runnable from any position, so the first
    # value a plan commands must always appear as a move.
    last_external: dict[str, float] = {}
    arm_baseline, rail_baseline = _joint_baselines(items, plan)

    for index, step in enumerate(plan.steps):
        name = f"{base_name} {index + 1:02d} {step.label}"
        target = _add_joint_target(
            rdk,
            name,
            frame,
            items.robot,
            _full_joint_vector(items, plan, index, arm_baseline, rail_baseline),
        )
        targets.append(target)
        program.MoveJ(target)

        for rail_map in external:
            value = getattr(step.rail, rail_map.axis, None)
            if value is None or last_external.get(rail_map.axis) == value:
                continue
            last_external[rail_map.axis] = value

            rail_item = items.rail_items[rail_map.axis]
            rail_target = _add_joint_target(
                rdk,
                f"{name} {rail_map.axis}={value:.0f}",
                rail_frames[rail_map.axis],
                rail_item,
                [rail_map.to_robodk(value)],
            )
            rail_targets[rail_map.axis].append(rail_target)
            rail_programs[rail_map.axis].MoveJ(rail_target)
            program.RunInstruction(
                f"{rail_map.axis} rail -> {value:.0f}mm ({rail_item.Name()})",
                robolink.INSTRUCTION_COMMENT,
            )

    # Same gap as the joint baselines: if no step commands an external rail because the
    # plan starts where it already needs to be, the companion program would come out
    # empty. Pin it to the position the plan assumed instead.
    for rail_map in external:
        axis = rail_map.axis
        if rail_targets[axis]:
            continue
        assumed = getattr(plan.start_rail, axis, None)
        if assumed is None:
            continue
        rail_item = items.rail_items[axis]
        rail_target = _add_joint_target(
            rdk,
            f"{base_name} 00 start {axis}={assumed:.0f}",
            rail_frames[axis],
            rail_item,
            [rail_map.to_robodk(assumed)],
        )
        rail_targets[axis].append(rail_target)
        rail_programs[axis].MoveJ(rail_target)

    for axis, rail_program in rail_programs.items():
        notes.append(
            f"{axis} rail is a separate mechanism, so its moves are in "
            f"{rail_program.Name()!r}; RoboDK cannot interleave two mechanisms in one "
            "program. The main program marks each value as a comment."
        )
    for warning in plan.warnings:
        notes.append(f"planner warning: {warning}")

    return ExportResult(
        frame=frame,
        program=program,
        targets=targets,
        rail_programs=rail_programs,
        rail_targets=rail_targets,
        notes=notes,
    )
