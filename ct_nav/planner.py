"""Turn a chosen navigation node into a flat list of moves.

Reproduces how an MHR actually reaches a target. The arm first retracts to the park
pose its target tree hangs off, because the rails must not travel with the arm
extended. The rails then walk the ``highway_tree`` from wherever the arm currently
is to the target's ``highway_node``, hop by hop, since the highway exists precisely
to break long rail moves into safe segments. A last rail move goes to the target's
own ``rail_pose``. Only then does the arm walk down the arm tree from the park pose
through every ``parent`` node to the selected node.

A pick or place is that inbound walk plus the same chain reversed back to park: the
arm must not stay at the leaf, because the next rail move is unsafe with it
extended. ``Visit.PICK_PLACE`` is that round trip; ``Visit.ENTER`` / ``Visit.EXIT``
are the two halves, kept for inspecting a single pose.

Both the live driver and the program exporter consume the same ``Plan.steps``, so a
previewed move and an exported program cannot drift apart.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .config import ArmConfig, NavTarget, NavTree, RailPose, TreeNode
from .park_poses import is_park_name, normalize_park_name, park_joints

# mhr_xz's cluster_config.yaml notes "Due to the rail position wrap, rail moves need
# to stay under 8190mm" and inserts extra highway nodes to break long moves up. A
# planned hop longer than this means the highway is being bypassed, so warn rather
# than silently emit a move the real rail would refuse.
MAX_RAIL_MOVE_MM = 8190.0

# Destinations you walk to and stay at. Reversing them would undo the tree: tuck_away
# exists so the arm ends at tucked_away, not back at the orthogonal park it left.
STAY_NODES = frozenset({"tucked_away"})
STAY_TARGETS = frozenset({"tuck_away"})


class PlanError(Exception):
    """Raised when a target cannot be resolved into a sequence of moves."""


class Mode(str, Enum):
    """How much of the real navigation to reproduce."""

    FULL = "full"
    ARM_ONLY = "arm_only"
    JUMP = "jump"

    @property
    def label(self) -> str:
        return {
            Mode.FULL: "Full navigation",
            Mode.ARM_ONLY: "Arm tree only",
            Mode.JUMP: "Jump to end pose",
        }[self]


class Visit(str, Enum):
    """Which half of the arm-tree walk to play, or both.

    Every usable pick/place is a walk down the tree to the leaf and the same walk
    reversed back to park. Staying at the leaf is only useful for inspecting a pose.
    """

    ENTER = "enter"
    EXIT = "exit"
    PICK_PLACE = "pick_place"

    @property
    def label(self) -> str:
        return {
            Visit.ENTER: "Enter only",
            Visit.EXIT: "Exit only",
            Visit.PICK_PLACE: "Pick / place (in and out)",
        }[self]


class StepKind(str, Enum):
    HIGHWAY = "highway"
    RAIL = "rail"
    PARK = "park"
    ARM = "arm"
    EXIT = "exit"
    COMBINED = "combined"


@dataclass(frozen=True)
class MoveStep:
    """One commanded state: a rail target, an arm pose, or both.

    ``rail`` is always absolute and carries every axis the arm has, so a step can be
    applied without replaying the ones before it. An empty ``RailPose`` means the
    rails stay put; ``joints is None`` means the arm stays put.
    """

    kind: StepKind
    label: str
    rail: RailPose = field(default_factory=RailPose)
    joints: tuple[float, ...] | None = None

    def describe(self) -> str:
        parts = [f"[{self.kind.value}] {self.label}"]
        if not self.rail.is_empty():
            parts.append(f"rail {self.rail.describe()}")
        if self.joints is not None:
            parts.append("j " + " ".join(f"{v:.2f}" for v in self.joints))
        return "  ".join(parts)


@dataclass
class Plan:
    arm: str
    nav_target: NavTarget
    node: str
    mode: Mode
    visit: Visit
    steps: list[MoveStep]
    highway_route: list[str]
    start_highway_node: str
    end_highway_node: str
    park_pose: str
    # Where the planner assumed the rails were. A step whose rail target equals this is
    # dropped as a no-op, so consumers that need an absolute value for every axis (the
    # program exporter) have to read the assumption rather than infer it from the steps.
    start_rail: RailPose = field(default_factory=RailPose)
    warnings: list[str] = field(default_factory=list)

    @property
    def label(self) -> str:
        return f"{self.arm} {self.nav_target.label}.{self.node}"

    def describe(self) -> str:
        lines = [f"{self.label}  ({self.mode.label}, {self.visit.label})"]
        if self.highway_route:
            lines.append("highway: " + " -> ".join(self.highway_route))
        lines.extend(f"  {i + 1:>2}. {s.describe()}" for i, s in enumerate(self.steps))
        lines.extend(f"  ! {w}" for w in self.warnings)
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Highway routing
# ---------------------------------------------------------------------------

def _ancestors(arm: ArmConfig, name: str) -> list[str]:
    """``name`` then each parent up to the root."""
    chain: list[str] = []
    seen: set[str] = set()
    current: str | None = name
    while current is not None:
        if current in seen:
            raise PlanError(f"{arm.name}: highway_tree has a cycle at {current!r}")
        seen.add(current)
        chain.append(current)
        node = arm.highway.get(current)
        if node is None:
            raise PlanError(f"{arm.name}: unknown highway node {current!r}")
        current = node.parent
    return chain


def highway_route(arm: ArmConfig, start: str, goal: str) -> list[str]:
    """Node names from ``start`` to ``goal`` inclusive, through their common ancestor.

    ``highway_tree`` is a tree of rail waypoints, so the only route between two nodes
    is up to the lowest common ancestor and back down.
    """
    if start == goal:
        return [start]

    up = _ancestors(arm, start)
    down = _ancestors(arm, goal)
    down_index = {name: i for i, name in enumerate(down)}

    for climbed, name in enumerate(up):
        if name in down_index:
            # up[:climbed + 1] ends at the meeting point; descend the reversed tail.
            return up[: climbed + 1] + list(reversed(down[: down_index[name]]))

    raise PlanError(
        f"{arm.name}: highway nodes {start!r} and {goal!r} are in disconnected trees"
    )


# ---------------------------------------------------------------------------
# Arm tree walking
# ---------------------------------------------------------------------------

def arm_chain(tree: NavTree, node_name: str) -> tuple[list[TreeNode], str]:
    """Nodes from the tree root down to ``node_name``, plus the park pose they hang off."""
    node = tree.nodes.get(node_name)
    if node is None:
        raise PlanError(
            f"tree {tree.file}.{tree.name} has no node {node_name!r} "
            f"(have: {', '.join(sorted(tree.nodes))})"
        )

    chain: list[TreeNode] = []
    seen: set[str] = set()
    current: TreeNode | None = node
    while current is not None:
        if current.name in seen:
            raise PlanError(f"tree {tree.file}.{tree.name} has a cycle at {current.name!r}")
        seen.add(current.name)
        chain.append(current)

        parent = current.parent
        if parent is None:
            raise PlanError(
                f"tree {tree.file}.{tree.name}: node {current.name!r} has no parent and "
                "no park pose to start from"
            )
        if parent in tree.nodes:
            current = tree.nodes[parent]
            continue

        park = normalize_park_name(parent)
        if not park:
            raise PlanError(
                f"tree {tree.file}.{tree.name}: node {current.name!r} has parent "
                f"{parent!r}, which is neither a node in this tree nor a known park pose"
            )
        chain.reverse()
        return chain, park

    raise PlanError(f"tree {tree.file}.{tree.name}: could not resolve a root for {node_name!r}")


def _enter_arm_steps(chain: list[TreeNode]) -> list[MoveStep]:
    return [
        MoveStep(kind=StepKind.ARM, label=f"node {node.name}", joints=node.joints)
        for node in chain
    ]


def _exit_arm_steps(
    chain: list[TreeNode], park: tuple[float, ...], park_pose: str
) -> list[MoveStep]:
    """Walk back up the tree, skipping the leaf the arm is already standing on."""
    steps = [
        MoveStep(kind=StepKind.EXIT, label=f"exit {node.name}", joints=node.joints)
        for node in reversed(chain[:-1])
    ]
    steps.append(MoveStep(kind=StepKind.PARK, label=f"park {park_pose}", joints=park))
    return steps


def stays_at_node(node: str, target: str) -> bool:
    """True when the selected node is a destination, not a transient pick/place pose."""
    return node in STAY_NODES or target in STAY_TARGETS


def _retracts_after_enter(visit: Visit, node: str, target: str) -> bool:
    """Pick/place reverses back to park, except for stay destinations like tucked_away."""
    return visit is Visit.PICK_PLACE and not stays_at_node(node, target)


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------

def _check_rail(arm: ArmConfig, rail: RailPose, where: str, warnings: list[str]) -> None:
    for axis, value in rail.axes().items():
        limits = arm.rail_limits.get(axis)
        if limits is None:
            continue
        if not limits.contains(value):
            warnings.append(
                f"{where}: {axis}={value:.0f}mm is outside the {axis} rail travel bounds "
                f"[{limits.lower_mm:.0f}, {limits.upper_mm:.0f}]mm"
            )


def _rail_steps(
    arm: ArmConfig,
    route: list[str],
    final: RailPose,
    state: RailPose,
    warnings: list[str],
    *,
    skip_first: bool,
) -> tuple[list[MoveStep], RailPose]:
    """Rail moves along ``route`` then to ``final``, dropping ones that repeat an earlier move.

    Deduplication is against what an earlier step actually commanded, never against
    ``state`` -- the position the arm is *assumed* to already be at. That assumption
    comes from a highway node the operator picked and can simply be wrong, and some arms
    make it load-bearing: ``mhr_x``'s single highway node sits at the same x as most of
    its targets, so trusting the assumption would emit no rail move at all and leave the
    rail wherever it happened to be.
    """
    steps: list[MoveStep] = []
    commanded: RailPose | None = None

    plan_points: list[tuple[StepKind, str, RailPose]] = []
    for index, name in enumerate(route):
        if skip_first and index == 0:
            # The arm is standing on this node, and the next hop supersedes it anyway.
            continue
        plan_points.append((StepKind.HIGHWAY, f"highway {name}", arm.highway[name].rail_pose))
    plan_points.append((StepKind.RAIL, "rail pose", final))

    for kind, label, pose in plan_points:
        target = state.merged_with(pose)
        if target.is_empty() or target == commanded:
            continue
        _check_rail(arm, target, label, warnings)
        for axis, value in target.axes().items():
            previous = getattr(state, axis)
            if previous is not None and abs(value - previous) > MAX_RAIL_MOVE_MM:
                warnings.append(
                    f"{label}: {axis} move of {abs(value - previous):.0f}mm exceeds the "
                    f"{MAX_RAIL_MOVE_MM:.0f}mm rail wrap limit; the highway should break "
                    "this up"
                )
        steps.append(MoveStep(kind=kind, label=label, rail=target))
        state = target
        commanded = target

    return steps, state


def _initial_rail_state(arm: ArmConfig, start: str) -> RailPose:
    """Rail position implied by standing on highway node ``start``.

    Axes the start node does not command are unknown, which is exactly what a ``None``
    in ``RailPose`` means -- the first step that does command them will set them.
    """
    node = arm.highway.get(start)
    if node is None:
        raise PlanError(f"{arm.name}: unknown highway node {start!r}")
    return node.rail_pose


def _target_rail(arm: ArmConfig, nav_target: NavTarget) -> RailPose:
    """The complete absolute rail pose of a target.

    A target's ``rail_pose`` only lists the axes it cares about, so the axes it omits
    are filled in from its ``highway_node`` -- the position the arm would have reached
    by the time it arrives.
    """
    return _initial_rail_state(arm, nav_target.highway_node).merged_with(nav_target.rail_pose)


def plan_move(
    arm: ArmConfig,
    module: str,
    target: str,
    node: str,
    *,
    mode: Mode = Mode.FULL,
    visit: Visit = Visit.ENTER,
    current_highway_node: str | None = None,
) -> Plan:
    """Plan the moves that take ``arm`` to ``node`` of ``module.target``.

    ``mode`` and ``visit`` may be the enum or its string value. Accepting the string is
    not just convenience: both subclass ``str``, so a value that has been through a Qt
    QVariant (as it has when it comes out of a combo box's user data) arrives back as a
    plain ``str``. Coercing here means an identity comparison below cannot quietly
    mis-select the branch.
    """
    mode = Mode(mode)
    visit = Visit(visit)
    nav_target = arm.nav_target(module, target)

    if nav_target.highway_node not in arm.highway:
        raise PlanError(
            f"{arm.name}: {nav_target.label} points at highway node "
            f"{nav_target.highway_node!r}, which is not in highway_tree"
        )

    start = current_highway_node or arm.highway_root()
    if start not in arm.highway:
        raise PlanError(f"{arm.name}: unknown current highway node {start!r}")

    tree = arm.tree(nav_target)
    chain, park_pose = arm_chain(tree, node)

    warnings: list[str] = []
    steps: list[MoveStep] = []

    # Park at the arm's configured base (±90°) so rail travel happens in the same
    # tucked pose as Reset to PARK. The arm tree then swings J1 as needed.
    park = tuple(park_joints(park_pose, arm.location, arm.park_base_angle_deg))

    start_rail = RailPose()
    route = [nav_target.highway_node]

    if visit is Visit.EXIT:
        # Already at the leaf: rails stay put and the arm walks back to park.
        if mode is Mode.JUMP:
            steps.append(
                MoveStep(
                    kind=StepKind.COMBINED,
                    label=f"park {park_pose}",
                    joints=park,
                )
            )
        else:
            steps.extend(_exit_arm_steps(chain, park, park_pose))
    elif mode is Mode.JUMP:
        rail = _target_rail(arm, nav_target)
        _check_rail(arm, rail, "rail pose", warnings)
        steps.append(
            MoveStep(
                kind=StepKind.COMBINED,
                label=f"{nav_target.label}.{node}",
                rail=rail,
                joints=chain[-1].joints,
            )
        )
        if _retracts_after_enter(visit, node, nav_target.target):
            steps.append(MoveStep(kind=StepKind.PARK, label=f"park {park_pose}", joints=park))
    else:
        steps.append(MoveStep(kind=StepKind.PARK, label=f"park {park_pose}", joints=park))

        if mode is Mode.FULL:
            route = highway_route(arm, start, nav_target.highway_node)
            rail_route, final_rail = route, nav_target.rail_pose
            rail_state = _initial_rail_state(arm, start)
            start_rail = rail_state
        else:
            # ARM_ONLY deliberately ignores the highway, so where the rails are now is
            # unknown: start from a blank state so the one rail move is always emitted,
            # and command the target's complete pose rather than just its own axes.
            rail_route, final_rail = [], _target_rail(arm, nav_target)
            rail_state = RailPose()

        rail_steps, _ = _rail_steps(
            arm, rail_route, final_rail, rail_state, warnings, skip_first=mode is Mode.FULL
        )
        steps.extend(rail_steps)
        steps.extend(_enter_arm_steps(chain))
        if _retracts_after_enter(visit, node, nav_target.target):
            steps.extend(_exit_arm_steps(chain, park, park_pose))

    return Plan(
        arm=arm.name,
        nav_target=nav_target,
        node=node,
        mode=mode,
        visit=visit,
        steps=steps,
        highway_route=route,
        start_highway_node=start,
        end_highway_node=nav_target.highway_node,
        park_pose=park_pose,
        start_rail=start_rail,
        warnings=warnings,
    )


def plan_park(
    arm: ArmConfig,
    park_pose: str = "ORTHOGONAL_PARK",
    *,
    current_highway_node: str | None = None,
) -> Plan:
    """A single-step plan that retracts the arm to a park pose, rails untouched.

    Uses the arm's own ``park_base_angle`` (the same ±90° base as a navigation park).
    """
    if not is_park_name(park_pose):
        raise PlanError(f"Not a park pose: {park_pose!r}")
    joints = tuple(park_joints(park_pose, arm.location, arm.park_base_angle_deg))
    canonical = normalize_park_name(park_pose)
    # Parking does not move the rails, so the arm stays on whatever highway node it was on.
    root = current_highway_node or arm.highway_root()
    if root not in arm.highway:
        raise PlanError(f"{arm.name}: unknown current highway node {root!r}")
    return Plan(
        arm=arm.name,
        nav_target=NavTarget(
            module="-",
            target=canonical.lower(),
            rail_pose=RailPose(),
            tree_file="-",
            tree_name="-",
            highway_node=root,
        ),
        node=canonical,
        mode=Mode.ARM_ONLY,
        visit=Visit.EXIT,
        steps=[MoveStep(kind=StepKind.PARK, label=f"park {canonical}", joints=joints)],
        highway_route=[],
        start_highway_node=root,
        end_highway_node=root,
        park_pose=canonical,
        start_rail=arm.highway[root].rail_pose,
    )
