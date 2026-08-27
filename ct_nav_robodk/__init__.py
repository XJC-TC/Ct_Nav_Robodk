"""RoboDK bindings for the ct_config navigation model.

Everything that imports ``robodk`` lives here; ``ct_nav`` stays RoboDK-free so it can
be tested without an install.
"""

from .collision import CollisionHit, CollisionReport, EntityIndex
from .connection import active_station_name, connect
from .driver import Driver, DriverError, DriverOptions, nearest_highway_node, read_rail_pose
from .eoat import EoatError, EoatInventory, apply_eoat, list_eoats, visible_tooling_items
from .path_trace import PathMonitor, PathTraceError
from .program_export import ExportError, ExportResult, export_plan
from .station_map import (
    ArmMap,
    RailMap,
    StationMap,
    StationMapError,
    calibrate_offset,
    default_map_path,
    load_default_station_map,
    load_station_map,
    save_station_map,
    with_rail_offset,
)

__all__ = [
    "ArmMap",
    "CollisionHit",
    "CollisionReport",
    "Driver",
    "DriverError",
    "DriverOptions",
    "EoatError",
    "EoatInventory",
    "ExportError",
    "ExportResult",
    "EntityIndex",
    "PathMonitor",
    "PathTraceError",
    "RailMap",
    "StationMap",
    "StationMapError",
    "active_station_name",
    "calibrate_offset",
    "connect",
    "default_map_path",
    "export_plan",
    "apply_eoat",
    "list_eoats",
    "visible_tooling_items",
    "load_default_station_map",
    "load_station_map",
    "nearest_highway_node",
    "read_rail_pose",
    "save_station_map",
    "with_rail_offset",
]
