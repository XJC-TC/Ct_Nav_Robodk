"""ct_config navigation model, independent of RoboDK.

Loads a ct_config cluster and turns a chosen navigation node into a flat list of
moves. ``ct_nav_robodk`` is the only place that talks to RoboDK.
"""

from .config import (
    ArmConfig,
    ClusterConfig,
    ConfigError,
    HighwayNode,
    NavTarget,
    NavTree,
    RailLimits,
    RailPose,
    TreeNode,
    discover_arm_dirs,
    load_arm,
    load_cluster,
)
from .park_poses import PARK_NAMES, is_park_name, normalize_park_name, park_joints
from .planner import (
    MAX_RAIL_MOVE_MM,
    STAY_NODES,
    Mode,
    MoveStep,
    Plan,
    PlanError,
    StepKind,
    Visit,
    arm_chain,
    highway_route,
    plan_move,
    plan_park,
    stays_at_node,
)
from .units import UnitError, parse_deg, parse_mm

__all__ = [
    "MAX_RAIL_MOVE_MM",
    "PARK_NAMES",
    "STAY_NODES",
    "ArmConfig",
    "ClusterConfig",
    "ConfigError",
    "HighwayNode",
    "Mode",
    "MoveStep",
    "NavTarget",
    "NavTree",
    "Plan",
    "PlanError",
    "RailLimits",
    "RailPose",
    "StepKind",
    "TreeNode",
    "Visit",
    "UnitError",
    "arm_chain",
    "discover_arm_dirs",
    "highway_route",
    "is_park_name",
    "load_arm",
    "load_cluster",
    "normalize_park_name",
    "park_joints",
    "parse_deg",
    "parse_mm",
    "plan_move",
    "plan_park",
    "stays_at_node",
]
