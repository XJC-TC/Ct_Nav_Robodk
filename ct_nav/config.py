"""Load a ct_config cluster into plain dataclasses.

A cluster directory (e.g. ``D:/Bitbucket/ct_config/azula1``) holds one directory per
module. Only the arm modules (``mhr_*``) matter here; each contributes:

``cluster_config.yaml``
    ``highway_tree`` -- rail-only waypoints forming a tree via ``parent``;
    ``arm_nav_trees`` -- ``<module>.<target>`` entries pointing at an arm tree plus
    the rail pose and highway node to reach it; ``module_edges`` -- informational.
``navigation/<file>.yaml``
    ``trees.<tree>.<node>`` = ``{parent, pose}``, the joint-space nodes.
``ur10e.yaml`` / ``ur12e.yaml``
    ``location`` (which park pose table applies) and ``park_base_angle``.
``x_rail.yaml`` / ``z_rail.yaml``
    ``drive_attributes.travel_bounds``, used to reject impossible rail moves.

Nothing here imports RoboDK, so it is unit-testable against the real config.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import yaml

from .units import parse_deg, parse_mm

JOINT_ORDER = ("base", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3")

# Rail axes in station-tree order: x sits above z, which sits above the UR.
RAIL_AXES = ("x", "z")


class ConfigError(Exception):
    """Raised when a cluster directory does not look like usable ct_config."""


def _load_yaml(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise ConfigError(f"Cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"Invalid YAML in {path}: {exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f"Expected a mapping at the top level of {path}")
    return data


@dataclass(frozen=True)
class RailPose:
    """A rail position in ct_config module coordinates (mm), per axis.

    ``None`` means "this axis is not commanded here": ``mhr_x`` has no z rail, and
    the ``dummy`` highway node of the railless ``mhr_u*`` arms has an empty
    ``rail_pose``.
    """

    x: float | None = None
    z: float | None = None

    @classmethod
    def from_yaml(cls, raw: object, where: str) -> "RailPose":
        if raw is None:
            return cls()
        if not isinstance(raw, dict):
            raise ConfigError(f"{where}: rail_pose must be a mapping, got {type(raw).__name__}")
        unknown = set(raw) - set(RAIL_AXES)
        if unknown:
            raise ConfigError(f"{where}: unknown rail axes {sorted(unknown)}")
        return cls(
            x=parse_mm(raw["x"], f"{where}.x") if "x" in raw else None,
            z=parse_mm(raw["z"], f"{where}.z") if "z" in raw else None,
        )

    def axes(self) -> dict[str, float]:
        """Commanded axes only, in station-tree order."""
        return {
            axis: value
            for axis, value in ((a, getattr(self, a)) for a in RAIL_AXES)
            if value is not None
        }

    def is_empty(self) -> bool:
        return not self.axes()

    def merged_with(self, other: "RailPose") -> "RailPose":
        """``other``'s commanded axes win; axes it leaves out keep this pose's value."""
        return RailPose(
            x=other.x if other.x is not None else self.x,
            z=other.z if other.z is not None else self.z,
        )

    def describe(self) -> str:
        axes = self.axes()
        if not axes:
            return "-"
        return " ".join(f"{axis}={value:.0f}" for axis, value in axes.items())


@dataclass(frozen=True)
class HighwayNode:
    name: str
    parent: str | None
    rail_pose: RailPose


@dataclass(frozen=True)
class NavTarget:
    """One ``arm_nav_trees[module][target]`` entry."""

    module: str
    target: str
    rail_pose: RailPose
    tree_file: str
    tree_name: str
    highway_node: str

    @property
    def label(self) -> str:
        return f"{self.module}.{self.target}"


@dataclass(frozen=True)
class TreeNode:
    name: str
    parent: str | None
    joints: tuple[float, ...]


@dataclass(frozen=True)
class NavTree:
    file: str
    name: str
    nodes: dict[str, TreeNode]

    def roots(self) -> list[TreeNode]:
        """Nodes whose parent is not another node in this tree (i.e. a park pose)."""
        return [n for n in self.nodes.values() if n.parent not in self.nodes]

    def leaves(self) -> list[TreeNode]:
        """Nodes that nothing else in this tree lists as a parent -- the work poses."""
        parents = {n.parent for n in self.nodes.values() if n.parent in self.nodes}
        return [n for n in self.nodes.values() if n.name not in parents]

    def preferred_node(self) -> str | None:
        """The node an operator almost always wants: a pick/place leaf, else tucked_away.

        YAML insertion order puts approach nodes first (``enter``, ``enter_0``), so the
        combo box would otherwise land on those and stop short of the real destination.
        """
        if not self.nodes:
            return None
        for name in self.nodes:
            if name.startswith("pick_place_node"):
                return name
        if "tucked_away" in self.nodes:
            return "tucked_away"
        leaves = self.leaves()
        return (leaves[-1] if leaves else next(iter(self.nodes))).name


@dataclass(frozen=True)
class RailLimits:
    axis: str
    lower_mm: float
    upper_mm: float

    def contains(self, value_mm: float, tolerance_mm: float = 1e-6) -> bool:
        return self.lower_mm - tolerance_mm <= value_mm <= self.upper_mm + tolerance_mm


@dataclass
class ArmConfig:
    """One ``mhr_*`` directory: its highway, targets, trees and rail limits."""

    name: str
    path: Path
    location: str
    park_base_angle_deg: float
    robot_model: str
    highway: dict[str, HighwayNode]
    nav_targets: dict[str, dict[str, NavTarget]]
    trees: dict[str, NavTree] = field(default_factory=dict)
    rail_limits: dict[str, RailLimits] = field(default_factory=dict)

    # -- lookups -----------------------------------------------------------

    def modules(self) -> list[str]:
        return sorted(self.nav_targets)

    def targets(self, module: str) -> list[str]:
        return sorted(self.nav_targets.get(module, {}))

    def nav_target(self, module: str, target: str) -> NavTarget:
        try:
            return self.nav_targets[module][target]
        except KeyError as exc:
            raise ConfigError(f"{self.name}: no nav target {module}.{target}") from exc

    def tree(self, nav_target: NavTarget) -> NavTree:
        key = f"{nav_target.tree_file}.{nav_target.tree_name}"
        try:
            return self.trees[key]
        except KeyError as exc:
            raise ConfigError(
                f"{self.name}: {nav_target.label} points at missing tree "
                f"{key!r} (have: {', '.join(sorted(self.trees)) or 'none'})"
            ) from exc

    def rail_axes(self) -> tuple[str, ...]:
        """Axes this arm actually drives, in station-tree order (x above z)."""
        present = {
            axis
            for node in self.highway.values()
            for axis in node.rail_pose.axes()
        }
        present.update(
            axis
            for targets in self.nav_targets.values()
            for nav_target in targets.values()
            for axis in nav_target.rail_pose.axes()
        )
        return tuple(axis for axis in RAIL_AXES if axis in present)

    def highway_root(self) -> str:
        for node in self.highway.values():
            if node.parent is None:
                return node.name
        raise ConfigError(f"{self.name}: highway_tree has no root (no node with parent: null)")


@dataclass
class ClusterConfig:
    name: str
    path: Path
    arms: dict[str, ArmConfig]

    def arm_names(self) -> list[str]:
        return sorted(self.arms)

    def arm(self, name: str) -> ArmConfig:
        try:
            return self.arms[name]
        except KeyError as exc:
            raise ConfigError(
                f"{self.name}: no arm {name!r} (have: {', '.join(self.arm_names())})"
            ) from exc


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def _parse_highway(raw: object, where: str) -> dict[str, HighwayNode]:
    if not isinstance(raw, dict) or not raw:
        raise ConfigError(f"{where}: highway_tree is missing or empty")

    nodes: dict[str, HighwayNode] = {}
    for name, body in raw.items():
        body = body or {}
        if not isinstance(body, dict):
            raise ConfigError(f"{where}.{name}: expected a mapping")
        parent = body.get("parent")
        if parent is not None and not isinstance(parent, str):
            raise ConfigError(f"{where}.{name}.parent: expected a name or null")
        nodes[name] = HighwayNode(
            name=name,
            parent=parent,
            rail_pose=RailPose.from_yaml(body.get("rail_pose"), f"{where}.{name}.rail_pose"),
        )

    for node in nodes.values():
        if node.parent is not None and node.parent not in nodes:
            raise ConfigError(
                f"{where}.{node.name}.parent: {node.parent!r} is not a highway node"
            )
    return nodes


def _parse_nav_targets(raw: object, where: str) -> dict[str, dict[str, NavTarget]]:
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ConfigError(f"{where}: arm_nav_trees must be a mapping")

    out: dict[str, dict[str, NavTarget]] = {}
    for module, targets in raw.items():
        if not isinstance(targets, dict):
            raise ConfigError(f"{where}.{module}: expected a mapping of targets")
        parsed: dict[str, NavTarget] = {}
        for target, body in targets.items():
            body = body or {}
            spot = f"{where}.{module}.{target}"
            if not isinstance(body, dict):
                raise ConfigError(f"{spot}: expected a mapping")

            pointer = body.get("arm_tree_pointer")
            if not isinstance(pointer, str) or "." not in pointer:
                raise ConfigError(
                    f"{spot}.arm_tree_pointer: expected '<tree_file>.<tree_name>', got {pointer!r}"
                )
            # Tree names can contain dots in principle; only the first one separates
            # the file from the tree (e.g. "rh_grex_1_icm_tree.drawer_0/1").
            tree_file, tree_name = pointer.split(".", 1)

            highway_node = body.get("highway_node")
            if not isinstance(highway_node, str):
                raise ConfigError(f"{spot}.highway_node: expected a highway node name")

            parsed[target] = NavTarget(
                module=module,
                target=target,
                rail_pose=RailPose.from_yaml(body.get("rail_pose"), f"{spot}.rail_pose"),
                tree_file=tree_file,
                tree_name=tree_name,
                highway_node=highway_node,
            )
        out[module] = parsed
    return out


def _parse_joints(raw: object, where: str) -> tuple[float, ...]:
    if not isinstance(raw, dict):
        raise ConfigError(f"{where}: pose must be a mapping of joint angles")
    missing = [name for name in JOINT_ORDER if name not in raw]
    if missing:
        raise ConfigError(f"{where}: pose is missing {missing}")
    unknown = set(raw) - set(JOINT_ORDER)
    if unknown:
        raise ConfigError(f"{where}: pose has unknown joints {sorted(unknown)}")
    return tuple(parse_deg(raw[name], f"{where}.{name}") for name in JOINT_ORDER)


def _parse_tree_file(path: Path) -> dict[str, NavTree]:
    data = _load_yaml(path)
    trees = data.get("trees")
    if trees is None:
        return {}
    if not isinstance(trees, dict):
        raise ConfigError(f"{path}: 'trees' must be a mapping")

    stem = path.stem
    out: dict[str, NavTree] = {}
    for tree_name, raw_nodes in trees.items():
        if not isinstance(raw_nodes, dict):
            raise ConfigError(f"{path}: trees.{tree_name} must be a mapping of nodes")
        nodes: dict[str, TreeNode] = {}
        for node_name, body in raw_nodes.items():
            body = body or {}
            where = f"{path.name}:trees.{tree_name}.{node_name}"
            if not isinstance(body, dict):
                raise ConfigError(f"{where}: expected a mapping")
            parent = body.get("parent")
            if parent is not None:
                parent = str(parent).strip().strip("'\"")
            nodes[node_name] = TreeNode(
                name=node_name,
                parent=parent,
                joints=_parse_joints(body.get("pose"), where),
            )
        out[f"{stem}.{tree_name}"] = NavTree(file=stem, name=tree_name, nodes=nodes)
    return out


def _parse_rail_limits(path: Path, axis: str) -> RailLimits | None:
    data = _load_yaml(path)
    bounds = (data.get("drive_attributes") or {}).get("travel_bounds")
    if not isinstance(bounds, dict):
        return None
    return RailLimits(
        axis=axis,
        lower_mm=parse_mm(bounds.get("lower", 0), f"{path.name}.travel_bounds.lower"),
        upper_mm=parse_mm(bounds.get("upper", 0), f"{path.name}.travel_bounds.upper"),
    )


def _find_robot_file(arm_dir: Path) -> Path | None:
    # mhr_x/mhr_xz carry a UR12e, mhr_u1/mhr_u2 a UR10e; other clusters may differ.
    candidates = sorted(p for p in arm_dir.glob("ur*.yaml") if p.is_file())
    return candidates[0] if candidates else None


def load_arm(arm_dir: Path) -> ArmConfig:
    """Load a single ``mhr_*`` directory."""
    arm_dir = Path(arm_dir)
    cluster_config = arm_dir / "cluster_config.yaml"
    if not cluster_config.is_file():
        raise ConfigError(f"{arm_dir}: no cluster_config.yaml")

    data = _load_yaml(cluster_config)
    where = cluster_config.name

    location = "lower"
    park_base_angle = 0.0
    robot_model = ""
    robot_file = _find_robot_file(arm_dir)
    if robot_file is not None:
        robot_model = robot_file.stem
        robot_data = _load_yaml(robot_file)
        raw_location = str(robot_data.get("location", "lower")).strip().lower()
        if raw_location not in ("lower", "upper"):
            raise ConfigError(
                f"{robot_file.name}: location must be 'lower' or 'upper', got {raw_location!r}"
            )
        location = raw_location
        if "park_base_angle" in robot_data:
            park_base_angle = parse_deg(
                robot_data["park_base_angle"], f"{robot_file.name}.park_base_angle"
            )

    trees: dict[str, NavTree] = {}
    nav_dir = arm_dir / "navigation"
    if nav_dir.is_dir():
        for tree_path in sorted(nav_dir.glob("*.yaml")):
            trees.update(_parse_tree_file(tree_path))

    rail_limits: dict[str, RailLimits] = {}
    for axis in RAIL_AXES:
        rail_file = arm_dir / f"{axis}_rail.yaml"
        if rail_file.is_file():
            limits = _parse_rail_limits(rail_file, axis)
            if limits is not None:
                rail_limits[axis] = limits

    return ArmConfig(
        name=arm_dir.name,
        path=arm_dir,
        location=location,
        park_base_angle_deg=park_base_angle,
        robot_model=robot_model,
        highway=_parse_highway(data.get("highway_tree"), where),
        nav_targets=_parse_nav_targets(data.get("arm_nav_trees"), where),
        trees=trees,
        rail_limits=rail_limits,
    )


def discover_arm_dirs(cluster_dir: Path) -> list[Path]:
    """Arm directories in a cluster: any child with a cluster_config.yaml.

    Matching on the file rather than an ``mhr_*`` name prefix keeps this working for
    clusters that name their arm module differently (``chiron/mhr``, ``mac/mhr_u``).
    """
    cluster_dir = Path(cluster_dir)
    if not cluster_dir.is_dir():
        raise ConfigError(f"Not a directory: {cluster_dir}")
    return sorted(
        child
        for child in cluster_dir.iterdir()
        if child.is_dir() and (child / "cluster_config.yaml").is_file()
    )


def load_cluster(cluster_dir: Path, arm_names: Iterable[str] | None = None) -> ClusterConfig:
    """Load every arm in ``cluster_dir`` (or only ``arm_names``)."""
    cluster_dir = Path(cluster_dir)
    wanted = set(arm_names) if arm_names is not None else None

    arms: dict[str, ArmConfig] = {}
    for arm_dir in discover_arm_dirs(cluster_dir):
        if wanted is not None and arm_dir.name not in wanted:
            continue
        arms[arm_dir.name] = load_arm(arm_dir)

    if not arms:
        raise ConfigError(
            f"{cluster_dir}: no arm directories with a cluster_config.yaml were found"
        )
    return ClusterConfig(name=cluster_dir.name, path=cluster_dir, arms=arms)
