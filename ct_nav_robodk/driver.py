"""Apply a ``Plan``'s steps to a live RoboDK station.

Deliberately drives the station with ``setJoints`` rather than ``MoveJ``: the point of
the panel is to watch a navigation sequence and confirm the poses in ct_config are what
the modeller expects, which wants immediate, interruptible motion with no program or
target left behind. Exporting a real program is ``program_export``'s job.

Rails are resolved through the ``StationMap``, so the three shapes NABOO-01 uses -- an
extra joint of the arm's own robot, an independent 1-axis mechanism, and (for other
stations) a translated frame -- all look the same to the caller.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterable, Iterator, Sequence

from robodk import robolink

from ct_nav.config import ArmConfig, RailPose
from ct_nav.planner import MoveStep, Plan

from .station_map import ArmMap, RailMap, StationMap, StationMapError

# Interpolating in fixed-size increments keeps a 4-metre rail traverse and a 20-degree
# wrist tweak moving at comparable on-screen speeds instead of both taking one frame.
DEG_PER_FRAME = 3.0
MM_PER_FRAME = 25.0
MAX_FRAMES_PER_STEP = 400
FRAME_INTERVAL_S = 1.0 / 60.0

_FRAME_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


class DriverError(Exception):
    """Raised when the station does not match the map, or a move cannot be applied."""


@dataclass
class ArmItems:
    """The station items one arm needs, resolved once per run."""

    arm_map: ArmMap
    robot: robolink.Item
    rail_items: dict[str, robolink.Item]

    def rail(self, axis: str) -> tuple[RailMap, robolink.Item | None]:
        rail_map = self.arm_map.rails.get(axis)
        if rail_map is None:
            raise DriverError(
                f"{self.arm_map.name}: the station map has no {axis} rail, but a step "
                "commands one"
            )
        return rail_map, self.rail_items.get(axis)


def _require(rdk: robolink.Robolink, name: str, item_type: int, what: str) -> robolink.Item:
    item = rdk.Item(name, item_type)
    if not item.Valid():
        raise DriverError(f"Station has no {what} named {name!r}")
    return item


def resolve_arm(rdk: robolink.Robolink, arm_map: ArmMap) -> ArmItems:
    """Look up every station item ``arm_map`` refers to and sanity-check the joint count."""
    robot = _require(rdk, arm_map.robot_item, robolink.ITEM_TYPE_ROBOT, "robot")

    dof = len(robot.Joints().list())
    if dof != arm_map.total_joint_count:
        raise DriverError(
            f"{arm_map.robot_item!r} has {dof} joints but the station map expects "
            f"{arm_map.total_joint_count} ({arm_map.ur_joint_count} arm joints + "
            f"{len(arm_map.robot_axis_rails())} rail axes). Re-run scripts/inspect_station.py."
        )
    for rail in arm_map.robot_axis_rails():
        if rail.joint_index > dof:
            raise StationMapError(
                f"{arm_map.name}: {rail.axis} rail is mapped to joint {rail.joint_index} "
                f"but {arm_map.robot_item!r} only has {dof}"
            )

    rail_items: dict[str, robolink.Item] = {}
    for rail in arm_map.external_rails():
        item_type = (
            robolink.ITEM_TYPE_FRAME if rail.kind == "frame" else robolink.ITEM_TYPE_ROBOT
        )
        rail_items[rail.axis] = _require(
            rdk, rail.item, item_type, "mechanism" if rail.kind != "frame" else "frame"
        )

    return ArmItems(arm_map=arm_map, robot=robot, rail_items=rail_items)


# ---------------------------------------------------------------------------
# Reading and writing rail positions
# ---------------------------------------------------------------------------

def read_rail_robodk(items: ArmItems, axis: str) -> float:
    """The rail's current value in RoboDK units, before any ct_config conversion."""
    rail_map, item = items.rail(axis)
    if rail_map.kind == "robot_axis":
        return float(items.robot.Joints().list()[rail_map.joint_index - 1])
    if item is None:
        raise DriverError(f"{items.arm_map.name}: {axis} rail item was not resolved")
    if rail_map.kind == "mechanism":
        return float(item.Joints().list()[rail_map.joint_index - 1])
    return float(item.Pose().Pos()[_FRAME_AXIS_INDEX[rail_map.frame_axis]])


def read_rail_ct(items: ArmItems, axis: str) -> float:
    """The rail's current position expressed in ct_config millimetres."""
    rail_map, _ = items.rail(axis)
    return rail_map.to_ct(read_rail_robodk(items, axis))


def read_rail_pose(items: ArmItems) -> RailPose:
    """Every mapped rail of this arm, in ct_config millimetres."""
    values = {axis: read_rail_ct(items, axis) for axis in items.arm_map.rails}
    return RailPose(x=values.get("x"), z=values.get("z"))


def _set_external_rail(items: ArmItems, rail_map: RailMap, value_mm: float) -> None:
    item = items.rail_items.get(rail_map.axis)
    if item is None:
        raise DriverError(f"{items.arm_map.name}: {rail_map.axis} rail item was not resolved")
    target = rail_map.to_robodk(value_mm)

    if rail_map.kind == "mechanism":
        joints = item.Joints().list()
        joints[rail_map.joint_index - 1] = target
        item.setJoints(joints)
        return

    # A frame rail: keep the frame's orientation and only slide it along one axis.
    pose = item.Pose()
    position = pose.Pos()
    position[_FRAME_AXIS_INDEX[rail_map.frame_axis]] = target
    pose.setPos(position)
    item.setPose(pose)


def _joint_vector(items: ArmItems, arm_joints: Sequence[float], rail: RailPose) -> list[float]:
    """The full RoboDK joint vector: arm joints plus any rail carried as a robot axis.

    Rail axes the caller does not command keep whatever the robot is at now, so a step
    that only moves the arm leaves the rails alone.
    """
    arm_map = items.arm_map
    if len(arm_joints) != arm_map.ur_joint_count:
        raise DriverError(
            f"{arm_map.name}: expected {arm_map.ur_joint_count} arm joints, "
            f"got {len(arm_joints)}"
        )

    current = items.robot.Joints().list()
    vector = list(current)
    vector[: arm_map.ur_joint_count] = [float(v) for v in arm_joints]

    for rail_map in arm_map.robot_axis_rails():
        value_mm = getattr(rail, rail_map.axis, None)
        if value_mm is not None:
            vector[rail_map.joint_index - 1] = rail_map.to_robodk(value_mm)
    return vector


# ---------------------------------------------------------------------------
# Applying steps
# ---------------------------------------------------------------------------

@dataclass
class DriverOptions:
    animate: bool = True
    speed: float = 1.0  # >1 is faster; scales the interpolation increment
    pause_between_steps_s: float = 0.0


class Driver:
    """Applies plan steps to one arm of a live station."""

    def __init__(
        self,
        rdk: robolink.Robolink,
        station_map: StationMap,
        options: DriverOptions | None = None,
    ) -> None:
        self.rdk = rdk
        self.station_map = station_map
        self.options = options or DriverOptions()
        self._resolved: dict[str, ArmItems] = {}

    # -- setup -------------------------------------------------------------

    def items_for(self, arm_name: str) -> ArmItems:
        if arm_name not in self._resolved:
            self._resolved[arm_name] = resolve_arm(self.rdk, self.station_map.arm(arm_name))
        return self._resolved[arm_name]

    def forget(self, arm_name: str | None = None) -> None:
        """Drop cached item lookups, e.g. after the user switches station."""
        if arm_name is None:
            self._resolved.clear()
        else:
            self._resolved.pop(arm_name, None)

    def verify(self, arm_names: Iterable[str] | None = None) -> list[str]:
        """Resolve arms without moving anything; returns one message per problem."""
        names = list(arm_names) if arm_names is not None else sorted(self.station_map.arms)
        problems: list[str] = []
        for name in names:
            try:
                self.items_for(name)
            except (DriverError, StationMapError) as exc:
                problems.append(str(exc))
        return problems

    # -- motion ------------------------------------------------------------

    def apply_step(self, arm_name: str, step: MoveStep) -> None:
        """Command one step instantly, with no interpolation."""
        items = self.items_for(arm_name)

        # External rails move first: they sit above the arm in the station tree, so
        # driving them after the arm would drag an already-posed arm through space.
        for rail_map in items.arm_map.external_rails():
            value_mm = getattr(step.rail, rail_map.axis, None)
            if value_mm is not None:
                _set_external_rail(items, rail_map, value_mm)

        arm_joints = step.joints
        if arm_joints is None and step.rail.is_empty():
            return
        if arm_joints is None:
            # A rail-only step: hold the arm where it is.
            arm_joints = items.robot.Joints().list()[: items.arm_map.ur_joint_count]

        items.robot.setJoints(_joint_vector(items, arm_joints, step.rail))

    def iter_frames(self, plan: Plan) -> Iterator[tuple[int, MoveStep]]:
        """Yield ``(step_index, frame)`` pairs to apply in order.

        A generator rather than a blocking loop so a GUI can pump one frame per timer
        tick on its own thread, keeping every RoboDK call on a single thread while
        staying responsive. Interpolation reads the station's current state when each
        step begins, so the consumer must apply each frame before asking for the next.
        """
        yield from self.iter_step_frames(plan.arm, plan.steps)

    def iter_step_frames(
        self, arm_name: str, steps: Sequence[MoveStep]
    ) -> Iterator[tuple[int, MoveStep]]:
        """Same as ``iter_frames``, for an arbitrary list of steps on ``arm_name``."""
        for index, step in enumerate(steps):
            if not self.options.animate:
                yield index, step
                continue

            items = self.items_for(arm_name)
            start_arm = items.robot.Joints().list()[: items.arm_map.ur_joint_count]
            end_arm = list(step.joints) if step.joints is not None else list(start_arm)
            start_rail = read_rail_pose(items)
            end_rail = start_rail.merged_with(step.rail)

            frames = self._frame_count(start_arm, end_arm, start_rail, end_rail)
            if frames <= 1:
                yield index, step
                continue

            for frame in range(1, frames + 1):
                ratio = frame / frames
                yield index, MoveStep(
                    kind=step.kind,
                    label=step.label,
                    rail=_blend_rail(start_rail, end_rail, ratio),
                    joints=tuple(a + (b - a) * ratio for a, b in zip(start_arm, end_arm)),
                )

    def _frame_count(
        self,
        start_arm: Sequence[float],
        end_arm: Sequence[float],
        start_rail: RailPose,
        end_rail: RailPose,
    ) -> int:
        max_deg = max((abs(b - a) for a, b in zip(start_arm, end_arm)), default=0.0)
        max_mm = 0.0
        for axis in ("x", "z"):
            a, b = getattr(start_rail, axis), getattr(end_rail, axis)
            if a is not None and b is not None:
                max_mm = max(max_mm, abs(b - a))

        speed = max(self.options.speed, 0.05)
        frames = max(max_deg / (DEG_PER_FRAME * speed), max_mm / (MM_PER_FRAME * speed))
        return min(int(frames) + 1, MAX_FRAMES_PER_STEP)

    def run(
        self,
        plan: Plan,
        should_stop: Callable[[], bool] | None = None,
        on_step: Callable[[int, MoveStep], None] | None = None,
    ) -> int:
        """Execute ``plan`` synchronously; returns how many steps completed.

        A stop request takes effect between interpolation frames, so the arm halts
        part-way through a step rather than snapping to the end of it.
        """
        done = 0
        current = -1
        for index, frame in self.iter_frames(plan):
            if should_stop is not None and should_stop():
                return done
            if index != current:
                if current >= 0:
                    done = current + 1
                    if self.options.pause_between_steps_s > 0:
                        time.sleep(self.options.pause_between_steps_s)
                current = index
                if on_step is not None:
                    on_step(index, plan.steps[index])

            self.apply_step(plan.arm, frame)
            if self.options.animate:
                time.sleep(FRAME_INTERVAL_S)

        return len(plan.steps) if current >= 0 else done


def _blend_rail(start: RailPose, end: RailPose, ratio: float) -> RailPose:
    values: dict[str, float | None] = {}
    for axis in ("x", "z"):
        a, b = getattr(start, axis), getattr(end, axis)
        if b is None:
            values[axis] = None
        elif a is None:
            # Nothing to interpolate from; command the endpoint straight away.
            values[axis] = b
        else:
            values[axis] = a + (b - a) * ratio
    return RailPose(x=values["x"], z=values["z"])


def nearest_highway_node(arm_config: ArmConfig, items: ArmItems) -> str | None:
    """The highway node whose rail pose best matches where the arm currently stands.

    Lets the panel recover a sensible starting node after the user jogs the station by
    hand, instead of planning from the highway root and driving a long way backwards.
    """
    current = read_rail_pose(items)
    if current.is_empty():
        return None

    best_name: str | None = None
    best_distance = float("inf")
    for node in arm_config.highway.values():
        axes = node.rail_pose.axes()
        if not axes:
            continue
        distance = 0.0
        for axis, value in axes.items():
            actual = getattr(current, axis, None)
            if actual is None:
                continue
            distance += (value - actual) ** 2
        if distance < best_distance:
            best_distance, best_name = distance, node.name
    return best_name
