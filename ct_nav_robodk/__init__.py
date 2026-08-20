"""RoboDK bindings for the ct_config navigation model.

Everything that imports ``robodk`` lives here; ``ct_nav`` stays RoboDK-free so it can
be tested without an install.
"""

from .connection import active_station_name, connect
from .driver import Driver, DriverError, DriverOptions, nearest_highway_node, read_rail_pose
from .eoat import EoatError, EoatInventory, apply_eoat, list_eoats
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
    "Driver",
    "DriverError",
    "DriverOptions",
    "EoatError",
    "EoatInventory",
    "ExportError",
    "ExportResult",
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
    "load_default_station_map",
    "load_station_map",
    "nearest_highway_node",
    "read_rail_pose",
    "save_station_map",
    "with_rail_offset",
]
