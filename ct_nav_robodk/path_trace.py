"""Paint a full-body coloured cube trail and stop when a cube overlaps an entity.

Collision is ours: axis-aligned cubes vs cached station AABBs / other-arm cubes.
RoboDK's Collisions() engine is not used — it is too heavy for live playback.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from robodk import robomath, robolink

from .collision import (
    Aabb,
    CollisionHit,
    CollisionReport,
    CubeOccupancy,
    EntityIndex,
    aabb_from_points,
    bone_aabb,
    cube_aabb,
)
from .driver import ArmItems
from .eoat import live_payload_geometry
from .path_geom import (
    BLOCK_SIZE_MM,
    EOAT_FALLBACK_MAX,
    EOAT_FALLBACK_MIN,
    EOAT_VOXELS,
    FLUSH_CUBES,
    HIT_COLOR,
    LINK_RADIUS_MM,
    MAX_LINK_VOXELS,
    ROOT_FRAME,
    TRACE_PREFIX,
    BlockBuffer,
    LinkPoint,
    SizedCube,
    VoxelTrail,
    bone_samples,
    cap_voxels,
    centers_from_voxels,
    choose_local_mesh,
    color_for_arm,
    cubes_vertices_sized,
    dedup_points,
    fill_aabb,
    fill_ball,
    fit_cad_cubes,
    is_trace_name,
    object_name,
    swept_trail_centers,
    trail_stamp_due,
    ur10e_link_local_voxels,
)

# One GetPoints pass per Go, then playback is only AABB tests + batched AddShape.
_STATIC_INDEX_BUDGET_S = 0.25
_MAX_STATIC_ITEMS = 80
_MAX_MESH_POINTS = 12000
_MAX_MESH_FEATURES = 24


class PathTraceError(Exception):
    """Raised when a path-trace object cannot be created or updated."""


def _xyz(pose: robomath.Mat) -> tuple[float, float, float]:
    position = pose.Pos()
    return (float(position[0]), float(position[1]), float(position[2]))


def robot_link_points(robot: robolink.Item) -> list[LinkPoint]:
    """World-frame positions of each robot link, via ObjectLink (not composed FK)."""
    points: list[LinkPoint] = []
    try:
        njoints = len(robot.Joints().list())
    except Exception:
        njoints = 6
    for index in range(njoints + 1):
        try:
            link = robot.ObjectLink(index)
            if not link.Valid():
                continue
            points.append(LinkPoint(key=f"link{index}", xyz=_xyz(link.PoseAbs())))
        except Exception:
            continue
    return points


def collect_link_points(items: ArmItems) -> list[LinkPoint]:
    """World positions covering the whole arm: joints, bones, TCP, rails, visible tools."""
    robot = items.robot
    points = robot_link_points(robot)

    try:
        points.append(LinkPoint(key="tcp", xyz=_xyz(robot.PoseAbs())))
    except Exception:
        pass

    for axis, rail_item in items.rail_items.items():
        try:
            if rail_item is not None and rail_item.Valid():
                points.append(LinkPoint(key=f"rail_{axis}", xyz=_xyz(rail_item.PoseAbs())))
        except Exception:
            continue

    try:
        for tool in robot.getLinks(robolink.ITEM_TYPE_TOOL) or []:
            if not tool.Valid() or not tool.Visible():
                continue
            points.append(LinkPoint(key=f"tool:{tool.Name()}", xyz=_xyz(tool.PoseAbs())))
    except Exception:
        pass

    return bone_samples(dedup_points(points))


def _local_mesh_points(item: robolink.Item) -> list[tuple[float, float, float]]:
    """Mesh vertices in the item's local frame, including every shape ID on the object."""
    local: list[tuple[float, float, float]] = []
    for feature in (robolink.FEATURE_OBJECT_MESH, robolink.FEATURE_MESH, robolink.FEATURE_POINT):
        got_any = False
        for feature_id in range(_MAX_MESH_FEATURES):
            try:
                result = item.GetPoints(feature, feature_id)
            except Exception:
                break
            points = None
            if isinstance(result, tuple) and result and result[0]:
                points = result[0]
            if not points:
                break
            got_any = True
            for row in points:
                if len(row) < 3:
                    continue
                local.append((float(row[0]), float(row[1]), float(row[2])))
        if len(local) >= _MAX_MESH_POINTS:
            return local
        if got_any:
            break
    return local


def _points_in_local(item: robolink.Item) -> list[tuple[float, float, float]]:
    """Mesh vertices in the item's local frame. World-frame dumps are converted once."""
    points = _local_mesh_points(item)
    if len(points) < 3:
        return []
    try:
        converted = _pose_transform(item.PoseAbs().inv(), points)
    except Exception:
        return points
    return choose_local_mesh(points, converted)


def _next_in_local(link: robolink.Item, nxt: robolink.Item | None) -> tuple[float, float, float] | None:
    if nxt is None:
        return None
    try:
        rel = link.PoseAbs().inv() * nxt.PoseAbs()
        return _xyz(rel)
    except Exception:
        return None


def _link_local_cubes(
    link: robolink.Item,
    nxt: robolink.Item | None,
    *,
    index: int,
    last_index: int,
) -> list[SizedCube]:
    """Cubes in this ObjectLink frame hugging the CAD, or a UR10e tube+joint fallback."""
    cubes = fit_cad_cubes(_points_in_local(link), max_cubes=MAX_LINK_VOXELS)
    if cubes:
        return cubes
    cells = ur10e_link_local_voxels(
        _next_in_local(link, nxt), index=index, last_index=last_index, size_mm=40.0
    )
    return [
        SizedCube(center, 40.0)
        for center in centers_from_voxels(cap_voxels(cells, MAX_LINK_VOXELS), 40.0)
    ]


def _eoat_local_cubes(item: robolink.Item) -> list[SizedCube]:
    """Cubes covering this EOAT / cable-guard mesh in its own frame."""
    cubes = fit_cad_cubes(_points_in_local(item), max_cubes=EOAT_VOXELS)
    if cubes:
        return cubes
    fallback = fill_aabb(EOAT_FALLBACK_MIN, EOAT_FALLBACK_MAX, size_mm=40.0)
    fallback |= fill_ball((0.0, 0.0, 40.0), 40.0, size_mm=40.0)
    return [
        SizedCube(center, 40.0)
        for center in centers_from_voxels(cap_voxels(fallback, EOAT_VOXELS), 40.0)
    ]


def _pose_transform(
    pose: robomath.Mat, points: list[tuple[float, float, float]]
) -> list[tuple[float, float, float]]:
    """Apply a 4x4 pose to points in Python — no per-point RoboDK round-trip."""
    try:
        rows = pose.rows
        r00, r01, r02, px = float(rows[0][0]), float(rows[0][1]), float(rows[0][2]), float(rows[0][3])
        r10, r11, r12, py = float(rows[1][0]), float(rows[1][1]), float(rows[1][2]), float(rows[1][3])
        r20, r21, r22, pz = float(rows[2][0]), float(rows[2][1]), float(rows[2][2]), float(rows[2][3])
    except Exception:
        out: list[tuple[float, float, float]] = []
        for xyz in points:
            world = pose * robomath.xyzrpw_2_pose([xyz[0], xyz[1], xyz[2], 0, 0, 0])
            out.append(_xyz(world))
        return out
    transformed: list[tuple[float, float, float]] = []
    for x, y, z in points:
        transformed.append(
            (
                r00 * x + r01 * y + r02 * z + px,
                r10 * x + r11 * y + r12 * z + py,
                r20 * x + r21 * y + r22 * z + pz,
            )
        )
    return transformed


@dataclass
class RigPart:
    """Cube centres in a station item's local frame; the mesh is parented to that item."""

    host: robolink.Item
    local: list[SizedCube]
    shape: robolink.Item | None = None
    kind: str = "link"


def _ur_links(robot: robolink.Item, ur_joints: int) -> list[robolink.Item]:
    links: list[robolink.Item] = []
    for index in range(ur_joints + 1):
        try:
            link = robot.ObjectLink(index)
            if link.Valid():
                links.append(link)
        except Exception:
            continue
    return links


def collect_rig_parts(
    rdk: robolink.Robolink, items: ArmItems
) -> list[RigPart]:
    """One parented cube body per UR link, plus the visible EOAT and cable guard."""
    robot = items.robot
    ur_n = items.arm_map.ur_joint_count
    links = _ur_links(robot, ur_n)
    last = max(len(links) - 1, 0)
    parts: list[RigPart] = []
    for index, link in enumerate(links):
        nxt = links[index + 1] if index + 1 < len(links) else None
        local = _link_local_cubes(link, nxt, index=index, last_index=last)
        if local:
            parts.append(RigPart(host=link, local=local, kind="link"))
    eoat_found = False
    for item in live_payload_geometry(rdk, robot):
        local = _eoat_local_cubes(item)
        if not local:
            continue
        parts.append(RigPart(host=item, local=local, kind="eoat"))
        eoat_found = True
    if not eoat_found:
        host = None
        try:
            tool = robot.getLink(robolink.ITEM_TYPE_TOOL)
            if tool is not None and tool.Valid():
                host = tool
        except Exception:
            host = None
        if host is None and links:
            host = links[-1]
        if host is not None:
            local = [
                SizedCube(center, 40.0)
                for center in centers_from_voxels(
                    fill_aabb(EOAT_FALLBACK_MIN, EOAT_FALLBACK_MAX, size_mm=40.0)
                    | fill_ball((0.0, 0.0, 40.0), 40.0, size_mm=40.0),
                    40.0,
                )
            ]
            parts.append(RigPart(host=host, local=local, kind="eoat"))
    return parts


def world_cubes_from_rig(parts: list[RigPart]) -> list[SizedCube]:
    """Trail / collision samples: current world cubes of the parented live wrap."""
    cubes: list[SizedCube] = []
    for part in parts:
        if not part.local:
            continue
        try:
            pose = part.host.PoseAbs()
        except Exception:
            continue
        world = _pose_transform(pose, [cube.center for cube in part.local])
        for cube, center in zip(part.local, world):
            cubes.append(SizedCube(center, cube.size_mm))
    return cubes


def _paint_item(item: robolink.Item, color: tuple[float, float, float, float]) -> None:
    """Tint AddShape geometry. Recolor replaces vertex colours; setColor is the object tint."""
    rgba = [float(channel) for channel in color]
    try:
        item.Recolor(rgba)
    except Exception:
        pass
    try:
        item.setColor(rgba)
    except Exception:
        pass


def _colored_shape(vertices: list[tuple[float, float, float]], color: tuple[float, float, float, float]):
    """AddShape payload: a 3xN triangle matrix plus RGBA, so the mesh is not default grey."""
    matrix = robomath.Mat([list(vertex) for vertex in vertices]).tr()
    return [matrix, [float(channel) for channel in color]]


def _under_any_robot(item: robolink.Item) -> bool:
    """True for robot CAD / tools / links. Those are the moving bodies, not cell obstacles."""
    current = item
    for _ in range(24):
        try:
            if current.Type() == robolink.ITEM_TYPE_ROBOT:
                return True
            current = current.Parent()
        except Exception:
            return False
        if not current.Valid():
            return False
        try:
            if current.Type() == robolink.ITEM_TYPE_STATION:
                return False
        except Exception:
            return False
    return False


def own_arm_aabbs(items: ArmItems, radius_mm: float = LINK_RADIUS_MM) -> list[Aabb]:
    """Current envelope of the moving arm — cubes sitting here are not collisions."""
    boxes: list[Aabb] = []
    links = robot_link_points(items.robot)
    if len(links) < 2:
        for point in links:
            boxes.append(cube_aabb(point.xyz, radius_mm * 2.0, name="self"))
    else:
        for start, end in zip(links, links[1:]):
            boxes.append(bone_aabb(start.xyz, end.xyz, radius_mm, name="self"))
    for rail_item in items.rail_items.values():
        try:
            if rail_item is None or not rail_item.Valid():
                continue
            boxes.append(cube_aabb(_xyz(rail_item.PoseAbs()), radius_mm * 2.0, name="self"))
        except Exception:
            continue
    return boxes


def _ancestors_include(item: robolink.Item, ancestor: robolink.Item) -> bool:
    current = item
    ancestor_name = ancestor.Name()
    for _ in range(24):
        try:
            current = current.Parent()
        except Exception:
            return False
        if not current.Valid():
            return False
        if current == ancestor or current.Name() == ancestor_name:
            return True
        if current.Type() == robolink.ITEM_TYPE_STATION:
            return False
    return False


def _mesh_aabb(item: robolink.Item, name: str) -> Aabb | None:
    """Local mesh points → world AABB from the 8 corners of the local box.

    GetPoints is used once per object at Go, never per frame. A miss is skipped rather
    than guessed, so we do not invent huge boxes that false-stop every move.
    """
    points = None
    for feature in (robolink.FEATURE_OBJECT_MESH, robolink.FEATURE_MESH, robolink.FEATURE_POINT):
        try:
            result = item.GetPoints(feature, 0)
        except Exception:
            continue
        if isinstance(result, tuple) and result and result[0]:
            points = result[0]
            break
    if not points:
        return None

    local: list[tuple[float, float, float]] = []
    for row in points[:_MAX_MESH_POINTS]:
        if len(row) < 3:
            continue
        local.append((float(row[0]), float(row[1]), float(row[2])))
    local_box = aabb_from_points(local)
    if local_box is None:
        return None

    try:
        pose = item.PoseAbs()
    except Exception:
        return Aabb(local_box.minimum, local_box.maximum, name=name)

    corners = []
    for x in (local_box.minimum[0], local_box.maximum[0]):
        for y in (local_box.minimum[1], local_box.maximum[1]):
            for z in (local_box.minimum[2], local_box.maximum[2]):
                world = pose * robomath.xyzrpw_2_pose([x, y, z, 0, 0, 0])
                corners.append(_xyz(world))
    return aabb_from_points(corners, name=name)


def build_static_index(rdk: robolink.Robolink, moving: robolink.Item) -> list[Aabb]:
    """Visible cell objects/tools that are not part of the moving arm, with a time budget."""
    boxes: list[Aabb] = []
    deadline = time.perf_counter() + _STATIC_INDEX_BUDGET_S
    counted = 0
    for kind in (robolink.ITEM_TYPE_OBJECT, robolink.ITEM_TYPE_TOOL):
        try:
            items = rdk.ItemList(kind) or []
        except Exception:
            continue
        for item in items:
            if time.perf_counter() > deadline or counted >= _MAX_STATIC_ITEMS:
                return boxes
            try:
                if not item.Valid() or not item.Visible():
                    continue
                name = item.Name()
            except Exception:
                continue
            if (
                is_trace_name(name)
                or _ancestors_include(item, moving)
                or _under_any_robot(item)
            ):
                continue
            box = _mesh_aabb(item, name)
            if box is None or not _plausible_aabb(box):
                continue
            boxes.append(box)
            counted += 1
    return boxes


def _plausible_aabb(box: Aabb) -> bool:
    """Drop dust-speck boxes and ones that span the whole cell (those false-stop every move)."""
    longest = max(box.maximum[i] - box.minimum[i] for i in range(3))
    return 8.0 <= longest <= 8000.0


def other_robot_bones(
    rdk: robolink.Robolink,
    moving: robolink.Item,
    skip_names: set[str] | None = None,
) -> list[Aabb]:
    """Cheap per-frame stand-in for every other robot: capsules along its current links."""
    boxes: list[Aabb] = []
    skip = {moving.Name()}
    if skip_names:
        skip.update(skip_names)
    try:
        robots = rdk.ItemList(robolink.ITEM_TYPE_ROBOT) or []
    except Exception:
        return boxes
    for robot in robots:
        try:
            if not robot.Valid() or robot.Name() in skip:
                continue
            if not robot.Visible():
                continue
        except Exception:
            continue
        links = robot_link_points(robot)
        name = robot.Name()
        if len(links) < 2:
            for point in links:
                boxes.append(cube_aabb(point.xyz, LINK_RADIUS_MM * 2.0, name=name))
            continue
        for start, end in zip(links, links[1:]):
            boxes.append(bone_aabb(start.xyz, end.xyz, LINK_RADIUS_MM, name=name))
    return boxes


def _contact_names(
    cubes: list[tuple[LinkPoint, Aabb]],
    own: list[Aabb],
    index: EntityIndex,
) -> set[str]:
    """Entities already overlapping this arm at Go — not collisions, just the start pose."""
    names: set[str] = set()
    boxes = index.static + index.moving
    for _, cube in cubes:
        for box in boxes:
            if box.name and cube.overlaps(box):
                names.add(box.name)
    for own_box in own:
        for box in boxes:
            if box.name and own_box.overlaps(box):
                names.add(box.name)
    return names


def _names_overlapping(cube: Aabb, index: EntityIndex, names: set[str]) -> set[str]:
    if not names:
        return set()
    hit: set[str] = set()
    for box in index.static + index.moving:
        if box.name in names and cube.overlaps(box):
            hit.add(box.name)
    return hit


class PathTracer:
    """Live CAD wrap posed onto each link/EOAT, plus a coarse world-fixed trail."""

    def __init__(self, rdk: robolink.Robolink) -> None:
        self.rdk = rdk
        self._root: robolink.Item | None = None
        self._objects: dict[str, robolink.Item] = {}
        self._hit_objects: dict[str, robolink.Item] = {}
        self._trails: dict[str, VoxelTrail] = {}
        self._hit_buffers: dict[str, BlockBuffer] = {}
        self._rigs: dict[str, list[RigPart]] = {}
        self._last_world: dict[str, list[tuple[float, float, float]]] = {}
        self._stamp_ticks: dict[str, int] = {}

    def _frame(self) -> robolink.Item:
        if self._root is not None and self._root.Valid():
            return self._root
        frame = self.rdk.Item(ROOT_FRAME, robolink.ITEM_TYPE_FRAME)
        if not frame.Valid():
            frame = self.rdk.AddFrame(ROOT_FRAME)
            if not frame.Valid():
                raise PathTraceError(f"RoboDK refused to create frame {ROOT_FRAME!r}")
        self._root = frame
        return frame

    def _delete_prefixed(self, prefix: str) -> None:
        try:
            objects = list(self.rdk.ItemList(robolink.ITEM_TYPE_OBJECT) or [])
        except Exception:
            return
        for item in objects:
            try:
                if item.Valid() and item.Name().startswith(prefix):
                    item.Delete()
            except Exception:
                continue

    def painted_count(self, arm: str) -> int:
        trail = self._trails.get(arm)
        return trail.count if trail is not None else 0

    def begin_arm(self, arm: str) -> None:
        """Wipe live voxels and trail so a new Go starts from this pose."""
        self._delete_prefixed(f"{TRACE_PREFIX} {arm}")
        self._objects.pop(arm, None)
        self._hit_objects.pop(arm, None)
        self._rigs.pop(arm, None)
        self._trails[arm] = VoxelTrail()
        self._hit_buffers[arm] = BlockBuffer(sample_mm=0.0)
        self._last_world.pop(arm, None)
        self._stamp_ticks.pop(arm, None)

    def forget_payload(self, arm: str) -> None:
        """Rebuild the EOAT wrap the next time the rig is ensured."""
        self._delete_prefixed(f"{TRACE_PREFIX} {arm} live")
        self._rigs.pop(arm, None)
        self._last_world.pop(arm, None)
        self._stamp_ticks.pop(arm, None)

    def ensure_arm(self, arm: str) -> None:
        if arm not in self._trails:
            self._trails[arm] = VoxelTrail()
        if arm not in self._hit_buffers:
            self._hit_buffers[arm] = BlockBuffer(sample_mm=0.0)
        if arm not in self._objects:
            existing = self.rdk.Item(object_name(arm), robolink.ITEM_TYPE_OBJECT)
            if existing.Valid():
                self._objects[arm] = existing
        self._frame()

    def ensure_live(self, arm: str, items: ArmItems) -> list[SizedCube]:
        """Parent cube bodies to UR links + EOAT once; they follow the arm including the final pose."""
        self.ensure_arm(arm)
        parts = self._rigs.get(arm)
        if not parts:
            parts = collect_rig_parts(self.rdk, items)
            color = color_for_arm(arm)
            for index, part in enumerate(parts):
                if not part.local:
                    continue
                name = object_name(arm, live=f"{part.kind}{index}")
                try:
                    part.shape = self._add_colored_cubes(
                        part.local, color, name=name, parent=part.host
                    )
                except PathTraceError:
                    continue
            self._rigs[arm] = parts
            self.sync_live(arm)
        return world_cubes_from_rig(parts)

    def sync_live(self, arm: str) -> None:
        """Put each live wrap on its host's current world pose (does not parent to ObjectLink)."""
        for part in self._rigs.get(arm, []):
            if part.shape is None:
                continue
            try:
                if not part.shape.Valid() or not part.host.Valid():
                    continue
                pose = part.host.PoseAbs()
                if hasattr(part.shape, "setPoseAbs"):
                    part.shape.setPoseAbs(pose)
                else:
                    part.shape.setPose(pose)
            except Exception:
                continue

    def stamp_trail(
        self, arm: str, items: ArmItems, *, force: bool = False
    ) -> tuple[list[tuple[float, float, float]], list[SizedCube]]:
        """Keep the live wrap on the arm; stamp the world trail only when due."""
        self.ensure_live(arm, items)
        self.sync_live(arm)
        cubes = world_cubes_from_rig(self._rigs.get(arm, []))
        centers = [cube.center for cube in cubes]
        trail = self._trails[arm]
        ticks = self._stamp_ticks.get(arm, 0) + 1
        self._stamp_ticks[arm] = ticks
        if not trail_stamp_due(self._last_world.get(arm), centers, ticks=ticks, force=force):
            return [], cubes
        self._stamp_ticks[arm] = 0
        swept = swept_trail_centers(self._last_world.get(arm), centers, trail.size_mm)
        added = trail.absorb(swept)
        self._last_world[arm] = centers
        if trail.pending_cubes >= FLUSH_CUBES:
            self.flush_trail(arm)
        return added, cubes

    def seal(self, arm: str, items: ArmItems) -> None:
        """Force the current pose (start or end) into the trail and flush it."""
        self.stamp_trail(arm, items, force=True)
        self.flush_trail(arm)

    def flush_trail(self, arm: str) -> None:
        trail = self._trails.get(arm)
        if trail is None:
            return
        pending = trail.take_pending()
        if pending:
            self._flush_vertices(arm, pending, hits=False)

    def mark_hit(self, arm: str, point: LinkPoint) -> None:
        self.ensure_arm(arm)
        buffer = self._hit_buffers[arm]
        buffer.consider(LinkPoint(key=f"hit:{point.key}", xyz=point.xyz))
        vertices = buffer.take_pending()
        if vertices:
            self._flush_vertices(arm, vertices, hits=True)

    def _add_colored_cubes(
        self,
        cubes: list[SizedCube],
        color: tuple[float, float, float, float],
        *,
        name: str,
        parent: robolink.Item | None,
    ) -> robolink.Item:
        vertices = cubes_vertices_sized(cubes)
        if not vertices:
            raise PathTraceError(f"No voxels to paint for {name}")
        created = self._add_shape(vertices, color)
        created.setName(name)
        created.setVisible(True)
        _paint_item(created, color)
        try:
            created.setParentStatic(self._frame())
        except Exception:
            pass
        self._snap_to_host(created, parent)
        return created

    def _snap_to_host(self, shape: robolink.Item, host: robolink.Item | None) -> None:
        if host is None:
            return
        try:
            if not host.Valid():
                return
            pose = host.PoseAbs()
            if hasattr(shape, "setPoseAbs"):
                shape.setPoseAbs(pose)
            else:
                shape.setPose(pose)
        except Exception:
            return

    def _add_shape(
        self,
        vertices: list[tuple[float, float, float]],
        color: tuple[float, float, float, float],
        attach: robolink.Item | None = None,
    ) -> robolink.Item:
        payload = _colored_shape(vertices, color)
        try:
            created = (
                self.rdk.AddShape(payload)
                if attach is None
                else self.rdk.AddShape(payload, attach, override_shapes=False)
            )
        except Exception:
            created = None
        if created is not None and hasattr(created, "Valid") and created.Valid():
            return created
        xyz = [list(vertex) for vertex in vertices]
        fallback = (
            self.rdk.AddShape(xyz)
            if attach is None
            else self.rdk.AddShape(xyz, attach, override_shapes=False)
        )
        if fallback is None or not fallback.Valid():
            raise PathTraceError(
                f"AddShape failed ({len(vertices)} verts). "
                "Check the station tree for a new object."
            )
        return fallback

    def _flush_vertices(
        self,
        arm: str,
        vertices: list[tuple[float, float, float]],
        *,
        hits: bool,
    ) -> None:
        if not vertices:
            return
        color = HIT_COLOR if hits else color_for_arm(arm)
        store = self._hit_objects if hits else self._objects
        obj = store.get(arm)
        if obj is None or not obj.Valid():
            created = self._add_shape(vertices, color)
            created.setName(object_name(arm, hits=hits))
            created.setVisible(True)
            _paint_item(created, color)
            try:
                created.setParentStatic(self._frame())
            except Exception:
                pass
            store[arm] = created
            return
        appended = self._add_shape(vertices, color, attach=obj)
        if appended is None:
            raise PathTraceError(f"AddShape append failed for {arm}")

    def clear(self, arm: str | None = None) -> None:
        if arm is None:
            names = list(self._trails) or _listed_trace_arms(self.rdk)
            for name in names:
                self.clear(name)
            frame = self.rdk.Item(ROOT_FRAME, robolink.ITEM_TYPE_FRAME)
            if frame.Valid():
                frame.Delete()
            self._root = None
            self._objects.clear()
            self._hit_objects.clear()
            self._trails.clear()
            self._hit_buffers.clear()
            self._rigs.clear()
            self._last_world.clear()
            self._stamp_ticks.clear()
            return
        self._delete_prefixed(f"{TRACE_PREFIX} {arm}")
        self._objects.pop(arm, None)
        self._hit_objects.pop(arm, None)
        self._trails.pop(arm, None)
        self._hit_buffers.pop(arm, None)
        self._rigs.pop(arm, None)
        self._last_world.pop(arm, None)
        self._stamp_ticks.pop(arm, None)


def _listed_trace_arms(rdk: robolink.Robolink) -> list[str]:
    prefix = TRACE_PREFIX + " "
    found: list[str] = []
    try:
        objects = rdk.ItemList(robolink.ITEM_TYPE_OBJECT) or []
    except Exception:
        return found
    for item in objects:
        try:
            name = item.Name()
        except Exception:
            continue
        if not is_trace_name(name) or name.endswith(" hits") or " live " in name:
            continue
        arm = name[len(prefix) :] if name.startswith(prefix) else ""
        if arm:
            found.append(arm)
    return found


@dataclass
class PathMonitor:
    """Playback session: drop cubes as the arm moves, optionally stop on overlap."""

    rdk: robolink.Robolink
    tracer: PathTracer | None = None
    index: EntityIndex | None = None
    painted: int = 0
    last_error: str | None = None
    _arm: str | None = None
    _items: ArmItems | None = None
    _start_contacts: set[str] = field(default_factory=set)
    _contacts_seeded: bool = False
    _static_ready: bool = False

    def __post_init__(self) -> None:
        if self.tracer is None:
            self.tracer = PathTracer(self.rdk)
        if self.index is None:
            self.index = EntityIndex(cubes=CubeOccupancy(BLOCK_SIZE_MM))

    @property
    def arm(self) -> str | None:
        return self._arm

    def attach(self, arm: str, items: ArmItems, *, replace: bool = False) -> None:
        self._arm = arm
        self._items = items
        self._start_contacts.clear()
        self._contacts_seeded = False
        self._static_ready = False
        assert self.tracer is not None
        assert self.index is not None
        if replace:
            self.tracer.begin_arm(arm)
            self.index.cubes.clear_arm(arm)
        else:
            self.tracer.ensure_arm(arm)
            self.tracer.forget_payload(arm)
        try:
            self.tracer.stamp_trail(arm, items, force=True)
            self.tracer.flush_trail(arm)
        except PathTraceError as exc:
            self.last_error = str(exc)
        self.index.static = []

    def observe(self, *, check_collision: bool = False) -> CollisionReport:
        """Keep the live voxel body on the arm, stamp new occupied cells, then maybe stop."""
        if self._arm is None or self._items is None or self.tracer is None or self.index is None:
            return CollisionReport()

        if check_collision:
            if not self._static_ready:
                self.index.static = build_static_index(self.rdk, self._items.robot)
                self._static_ready = True
            skip_rails: set[str] = set()
            for rail_item in self._items.rail_items.values():
                try:
                    if rail_item is not None and rail_item.Valid():
                        skip_rails.add(rail_item.Name())
                except Exception:
                    continue
            self.index.moving = other_robot_bones(self.rdk, self._items.robot, skip_rails)

        try:
            new_centers, world = self.tracer.stamp_trail(self._arm, self._items)
        except PathTraceError as exc:
            self.last_error = str(exc)
            return CollisionReport()

        self.painted = self.tracer.painted_count(self._arm)
        if not check_collision:
            return CollisionReport()

        own = [cube_aabb(cube.center, cube.size_mm, name="self") for cube in world]
        if not own:
            own = own_arm_aabbs(self._items)

        checks: list[tuple[LinkPoint, Aabb]] = []
        for index, cube in enumerate(world):
            checks.append(
                (
                    LinkPoint(f"live{index}", cube.center),
                    cube_aabb(cube.center, cube.size_mm, name=f"live{index}"),
                )
            )
        for index, center in enumerate(new_centers):
            point = LinkPoint(f"voxel{index}", center)
            self.index.cubes.add(self._arm, center)
            checks.append((point, cube_aabb(center, BLOCK_SIZE_MM, name=point.key)))

        if not self._contacts_seeded:
            seed = [
                (
                    LinkPoint(f"seed{index}", cube.center),
                    cube_aabb(cube.center, cube.size_mm, name="self"),
                )
                for index, cube in enumerate(world)
            ]
            self._start_contacts = _contact_names(seed, own, self.index)
            self._contacts_seeded = True

        hit_point: LinkPoint | None = None
        hit: CollisionHit | None = None
        still: set[str] = set()
        for point, cube in checks:
            found = self.index.hit(
                cube,
                ignore_arm=self._arm,
                size_mm=BLOCK_SIZE_MM,
                skip_entities=self._start_contacts,
            )
            if found is None:
                still.update(_names_overlapping(cube, self.index, self._start_contacts))
                continue
            if found.entity in self._start_contacts:
                still.add(found.entity)
                continue
            hit_point, hit = point, found
            break

        if hit is None:
            for own_box in own:
                still.update(_names_overlapping(own_box, self.index, self._start_contacts))
            self._start_contacts &= still
            return CollisionReport()
        try:
            self.tracer.mark_hit(self._arm, hit_point)
        except PathTraceError as exc:
            self.last_error = str(exc)
        return CollisionReport(hits=(hit,))

    def stop(self) -> None:
        if self._arm is not None and self._items is not None and self.tracer is not None:
            try:
                self.tracer.seal(self._arm, self._items)
            except PathTraceError:
                pass
        self._arm = None
        self._items = None

    def clear(self, arm: str | None = None) -> None:
        if self.index is not None:
            if arm is None:
                self.index.cubes = CubeOccupancy(BLOCK_SIZE_MM)
            else:
                self.index.cubes.clear_arm(arm)
        if self.tracer is not None:
            self.tracer.clear(arm)
