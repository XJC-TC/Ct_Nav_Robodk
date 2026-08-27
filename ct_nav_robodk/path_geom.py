"""Pure geometry for the path-trace colour blocks. No RoboDK import.

Live wrap hugs CAD with mixed 20/40/80 mm cubes. The world trail is a coarser
unique-cell occupancy so a long run stays visible. The live tracer in
``path_trace`` only converts the vertices into station objects.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# RGBA 0-1. Saturated common hues; red is reserved for collision hits; never white.
# Alpha is applied via Recolor on the mesh (setColor does not tint AddShape vertices).
ARM_ALPHA = 0.40
ARM_COLORS: dict[str, tuple[float, float, float, float]] = {
    "mhr_xz": (0.05, 0.35, 1.00, ARM_ALPHA),  # blue
    "mhr_x": (1.00, 0.45, 0.00, ARM_ALPHA),  # orange
    "mhr_u1": (0.00, 0.72, 0.22, ARM_ALPHA),  # green
    "mhr_u2": (0.62, 0.12, 0.95, ARM_ALPHA),  # violet
}

DEFAULT_COLOR = (0.00, 0.75, 0.85, ARM_ALPHA)  # cyan
HIT_COLOR = (0.95, 0.08, 0.06, 0.85)

ROOT_FRAME = "CtNav Paths"
TRACE_PREFIX = "CtNav Path"

# World trail: coarse unique cells so a long mhr_xz sweep stays visible to the end.
BLOCK_SIZE_MM = 80.0
CUBE_DRAW_MM = BLOCK_SIZE_MM * 0.90
SAMPLE_MM = 80.0
MAX_BLOCKS_PER_ARM = 12000
MAX_LIVE_VOXELS = 500
MAX_CHAIN_VOXELS = 400
FLUSH_CUBES = 80
VERTICES_PER_CUBE = 36  # 12 triangles * 3
# Live wrap hugs visible CAD with mixed cube sizes (fine → coarse merge).
LIVE_CUBE_SIZES = (20.0, 40.0, 80.0)
LIVE_DRAW_SCALE = 0.90
MAX_CAD_SPAN_MM = 900.0
MAX_FINE_CELLS = 3000
MAX_LIVE_CUBES_PER_PART = 80
TRAIL_STAMP_EVERY_N = 3
TRAIL_STAMP_MOVE_MM = BLOCK_SIZE_MM
MESH_FRAME_MARGIN_MM = 50.0
# Physical UR10e: tubes Ø~85 mm, shoulder/elbow housings Ø~120 mm, wrist Ø~90 mm.
UR10E_TUBE_RADIUS_MM = 42.0
UR10E_JOINT_RADIUS_MM = 58.0
UR10E_WRIST_RADIUS_MM = 45.0
UR10E_BASE_RADIUS_MM = 70.0
UR10E_MAX_BONE_MM = 750.0
UR10E_MIN_BONE_MM = 40.0
TUBE_RADIUS_MM = UR10E_TUBE_RADIUS_MM
JOINT_RADIUS_MM = UR10E_JOINT_RADIUS_MM
LINK_RADIUS_MM = UR10E_TUBE_RADIUS_MM
VOXELS_PER_PART = 80
EOAT_VOXELS = 80
MAX_LINK_VOXELS = 50
# Coupler-sized local box, parented to the flange / current EOAT item.
EOAT_FALLBACK_MIN = (-45.0, -45.0, 0.0)
EOAT_FALLBACK_MAX = (45.0, 45.0, 90.0)

# Close enough that a tool pose and the last joint pose are the same point.
DEDUP_MM = 8.0


@dataclass(frozen=True)
class LinkPoint:
    """One sampled location on the arm: a link, a rail carriage, or a visible tool."""

    key: str
    xyz: tuple[float, float, float]


def color_for_arm(arm: str) -> tuple[float, float, float, float]:
    return ARM_COLORS.get(arm, DEFAULT_COLOR)


def object_name(arm: str, *, hits: bool = False, live: str | None = None) -> str:
    if live is not None:
        return f"{TRACE_PREFIX} {arm} live {live}"
    suffix = " hits" if hits else ""
    return f"{TRACE_PREFIX} {arm}{suffix}"


def is_trace_name(name: str) -> bool:
    """True for the path-trace frame or any colour-block object we create."""
    return name == ROOT_FRAME or name.startswith(TRACE_PREFIX)


def distance_mm(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def mean_xyz(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    if not points:
        return (0.0, 0.0, 0.0)
    scale = 1.0 / len(points)
    return (
        sum(p[0] for p in points) * scale,
        sum(p[1] for p in points) * scale,
        sum(p[2] for p in points) * scale,
    )


def choose_local_mesh(
    raw: list[tuple[float, float, float]],
    converted: list[tuple[float, float, float]],
    *,
    margin_mm: float = MESH_FRAME_MARGIN_MM,
    max_span_mm: float = MAX_CAD_SPAN_MM,
) -> list[tuple[float, float, float]]:
    """Pick the vertex cloud that lives in the item's local frame.

    GetPoints sometimes returns world coordinates. The cloud whose centroid sits
    closer to the origin is the local one. A converted cloud that explodes past
    ``max_span_mm`` is rejected.
    """
    if len(converted) != len(raw) or not raw:
        return raw
    if point_span_mm(converted) > max_span_mm >= point_span_mm(raw):
        return raw
    d_raw = distance_mm(mean_xyz(raw), (0.0, 0.0, 0.0))
    d_conv = distance_mm(mean_xyz(converted), (0.0, 0.0, 0.0))
    if d_conv + margin_mm < d_raw:
        return converted
    return raw


def trail_stamp_due(
    previous: list[tuple[float, float, float]] | None,
    current: list[tuple[float, float, float]],
    *,
    ticks: int,
    force: bool,
    every_n: int = TRAIL_STAMP_EVERY_N,
    move_mm: float = TRAIL_STAMP_MOVE_MM,
) -> bool:
    """True when the world trail should absorb the current pose."""
    if force or previous is None:
        return True
    if ticks >= every_n:
        return True
    if not current:
        return False
    if previous and len(previous) == len(current):
        moved = max(distance_mm(a, b) for a, b in zip(previous, current))
    else:
        moved = distance_mm(mean_xyz(previous or []), mean_xyz(current))
    return moved >= move_mm


def dedup_points(points: list[LinkPoint], tol_mm: float = DEDUP_MM) -> list[LinkPoint]:
    """Drop points that sit on top of an earlier one in the same sample (TCP vs flange)."""
    kept: list[LinkPoint] = []
    for point in points:
        if any(distance_mm(point.xyz, previous.xyz) < tol_mm for previous in kept):
            continue
        kept.append(point)
    return kept


def bone_samples(
    points: list[LinkPoint], step_mm: float = SAMPLE_MM
) -> list[LinkPoint]:
    """Joint samples plus points along each consecutive ``link*`` bone.

    The colour blocks have to stand in for the whole arm, not just the joint origins.
    """
    extra: list[LinkPoint] = []
    links = [p for p in points if p.key.startswith("link")]
    for start, end in zip(links, links[1:]):
        span = distance_mm(start.xyz, end.xyz)
        if span < step_mm:
            continue
        steps = max(int(span / step_mm), 1)
        for i in range(1, steps):
            t = i / steps
            extra.append(
                LinkPoint(
                    key=f"{start.key}-{end.key}@{i}",
                    xyz=(
                        start.xyz[0] + (end.xyz[0] - start.xyz[0]) * t,
                        start.xyz[1] + (end.xyz[1] - start.xyz[1]) * t,
                        start.xyz[2] + (end.xyz[2] - start.xyz[2]) * t,
                    ),
                )
            )
    return points + extra


def cube_triangles(
    center: tuple[float, float, float], size_mm: float = CUBE_DRAW_MM
) -> list[tuple[float, float, float]]:
    """36 vertices (12 triangles) for an axis-aligned cube centred on ``center``."""
    half = size_mm / 2.0
    cx, cy, cz = center
    corners = (
        (cx - half, cy - half, cz - half),
        (cx + half, cy - half, cz - half),
        (cx + half, cy + half, cz - half),
        (cx - half, cy + half, cz - half),
        (cx - half, cy - half, cz + half),
        (cx + half, cy - half, cz + half),
        (cx + half, cy + half, cz + half),
        (cx - half, cy + half, cz + half),
    )
    faces = (
        (0, 2, 1, 0, 3, 2),  # -Z
        (4, 5, 6, 4, 6, 7),  # +Z
        (0, 1, 5, 0, 5, 4),  # -Y
        (3, 7, 6, 3, 6, 2),  # +Y
        (0, 4, 7, 0, 7, 3),  # -X
        (1, 2, 6, 1, 6, 5),  # +X
    )
    vertices: list[tuple[float, float, float]] = []
    for face in faces:
        vertices.extend(corners[i] for i in face)
    return vertices


VoxelKey = tuple[int, int, int]


def voxel_key(
    xyz: tuple[float, float, float], size_mm: float = BLOCK_SIZE_MM
) -> VoxelKey:
    scale = float(size_mm)
    return (round(xyz[0] / scale), round(xyz[1] / scale), round(xyz[2] / scale))


def voxel_center(key: VoxelKey, size_mm: float = BLOCK_SIZE_MM) -> tuple[float, float, float]:
    scale = float(size_mm)
    return (key[0] * scale, key[1] * scale, key[2] * scale)


def fill_ball(
    center: tuple[float, float, float],
    radius_mm: float,
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """Voxel keys covering a ball around a joint / flange origin."""
    step = max(int(round(radius_mm / size_mm)), 0)
    origin = voxel_key(center, size_mm)
    cells: set[VoxelKey] = set()
    limit = step * step
    for dx in range(-step, step + 1):
        for dy in range(-step, step + 1):
            for dz in range(-step, step + 1):
                if dx * dx + dy * dy + dz * dz <= limit:
                    cells.add((origin[0] + dx, origin[1] + dy, origin[2] + dz))
    return cells


def fill_capsule(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius_mm: float,
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """Voxel keys covering a capsule between two points (one rigid link)."""
    cells = fill_ball(start, radius_mm, size_mm) | fill_ball(end, radius_mm, size_mm)
    span = distance_mm(start, end)
    steps = max(int(span / size_mm), 1)
    for i in range(1, steps):
        t = i / steps
        point = (
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        )
        cells |= fill_ball(point, radius_mm, size_mm)
    return cells


def voxels_along_chain(
    origins: list[tuple[float, float, float]],
    radius_mm: float,
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """Voxel keys covering capsules between consecutive origins (the whole kinematic chain)."""
    if not origins:
        return set()
    if len(origins) == 1:
        return fill_ball(origins[0], radius_mm, size_mm)
    cells: set[VoxelKey] = set()
    for start, end in zip(origins, origins[1:]):
        cells |= fill_capsule(start, end, radius_mm, size_mm)
    return cells


def voxels_at_joints(
    origins: list[tuple[float, float, float]],
    radius_mm: float,
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """Non-overlapping voxels covering a ball at each joint origin (UR housings, not the tubes)."""
    cells: set[VoxelKey] = set()
    for origin in origins:
        cells |= fill_ball(origin, radius_mm, size_mm)
    return cells


def ur10e_arm_voxels(
    origins: list[tuple[float, float, float]],
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """Low-resolution UR10e: arm-sized tubes plus joint housings. Skips rail-length bones."""
    if not origins:
        return set()
    cells: set[VoxelKey] = set()
    last = len(origins) - 1
    for index, origin in enumerate(origins):
        if index == 0:
            radius = UR10E_BASE_RADIUS_MM
        elif index >= last - 2:
            radius = UR10E_WRIST_RADIUS_MM
        else:
            radius = UR10E_JOINT_RADIUS_MM
        cells |= fill_ball(origin, radius, size_mm)
    for start, end in zip(origins, origins[1:]):
        span = distance_mm(start, end)
        if span < UR10E_MIN_BONE_MM or span > UR10E_MAX_BONE_MM:
            continue
        cells |= fill_capsule(start, end, UR10E_TUBE_RADIUS_MM, size_mm)
    return cells


def ur10e_link_local_voxels(
    next_xyz: tuple[float, float, float] | None,
    *,
    index: int,
    last_index: int,
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """UR10e cubes in one ObjectLink's local frame: housing at the origin, tube toward the next joint."""
    if index == 0:
        radius = UR10E_BASE_RADIUS_MM
    elif index >= last_index - 2:
        radius = UR10E_WRIST_RADIUS_MM
    else:
        radius = UR10E_JOINT_RADIUS_MM
    cells = fill_ball((0.0, 0.0, 0.0), radius, size_mm)
    if next_xyz is None:
        return cells
    span = distance_mm((0.0, 0.0, 0.0), next_xyz)
    if UR10E_MIN_BONE_MM <= span <= UR10E_MAX_BONE_MM:
        cells |= fill_capsule((0.0, 0.0, 0.0), next_xyz, UR10E_TUBE_RADIUS_MM, size_mm)
    return cells


def fill_aabb(
    minimum: tuple[float, float, float],
    maximum: tuple[float, float, float],
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    """Every voxel whose centre is inside the axis-aligned box (inclusive)."""
    i0, j0, k0 = voxel_key(minimum, size_mm)
    i1, j1, k1 = voxel_key(maximum, size_mm)
    if i0 > i1:
        i0, i1 = i1, i0
    if j0 > j1:
        j0, j1 = j1, j0
    if k0 > k1:
        k0, k1 = k1, k0
    if (i1 - i0 + 1) * (j1 - j0 + 1) * (k1 - k0 + 1) > 4000:
        return set()
    cells: set[VoxelKey] = set()
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            for k in range(k0, k1 + 1):
                cells.add((i, j, k))
    return cells


def voxelize_points(
    points: list[tuple[float, float, float]], size_mm: float = BLOCK_SIZE_MM
) -> set[VoxelKey]:
    return {voxel_key(point, size_mm) for point in points}


def cap_voxels(cells: set[VoxelKey], limit: int) -> set[VoxelKey]:
    if len(cells) <= limit:
        return cells
    ordered = sorted(cells)
    step = len(ordered) / limit
    return {ordered[int(index * step)] for index in range(limit)}


def centers_from_voxels(
    cells: set[VoxelKey], size_mm: float = BLOCK_SIZE_MM
) -> list[tuple[float, float, float]]:
    return [voxel_center(key, size_mm) for key in sorted(cells)]


def cubes_vertices(
    centers: list[tuple[float, float, float]], size_mm: float = CUBE_DRAW_MM
) -> list[tuple[float, float, float]]:
    vertices: list[tuple[float, float, float]] = []
    for center in centers:
        vertices.extend(cube_triangles(center, size_mm))
    return vertices


@dataclass(frozen=True)
class SizedCube:
    """One axis-aligned cube. Live wrap uses mixed sizes; the trail does not."""

    center: tuple[float, float, float]
    size_mm: float


def point_span_mm(points: list[tuple[float, float, float]]) -> float:
    if not points:
        return 0.0
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))


def _dilate_cells(cells: set[VoxelKey]) -> set[VoxelKey]:
    grown: set[VoxelKey] = set()
    for i, j, k in cells:
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    grown.add((i + di, j + dj, k + dk))
    return grown


def _aligned_origin(index: int, factor: int) -> int:
    return index - (index % factor)


def _extract_aligned(
    cells: set[VoxelKey], *, fine_mm: float, factor: int, size_mm: float
) -> tuple[list[SizedCube], set[VoxelKey]]:
    if factor <= 1:
        cubes = [SizedCube(voxel_center(key, fine_mm), size_mm) for key in sorted(cells)]
        return cubes, set()
    groups: dict[tuple[int, int, int], set[VoxelKey]] = {}
    for key in cells:
        origin = (
            _aligned_origin(key[0], factor),
            _aligned_origin(key[1], factor),
            _aligned_origin(key[2], factor),
        )
        groups.setdefault(origin, set()).add(key)
    need = factor**3
    cubes: list[SizedCube] = []
    leftover: set[VoxelKey] = set()
    half = (factor - 1) * 0.5
    for origin, members in groups.items():
        if len(members) == need:
            cubes.append(
                SizedCube(
                    (
                        (origin[0] + half) * fine_mm,
                        (origin[1] + half) * fine_mm,
                        (origin[2] + half) * fine_mm,
                    ),
                    size_mm,
                )
            )
        else:
            leftover |= members
    return cubes, leftover


def fit_cad_cubes(
    points: list[tuple[float, float, float]],
    *,
    sizes: tuple[float, ...] = LIVE_CUBE_SIZES,
    max_cubes: int = MAX_LIVE_CUBES_PER_PART,
    max_span_mm: float = MAX_CAD_SPAN_MM,
) -> list[SizedCube]:
    """Hug a vertex cloud with 20/40/80 mm cubes. Does not fill the AABB."""
    if len(points) < 3 or not sizes:
        return []
    if point_span_mm(points) > max_span_mm:
        return []
    fine = sizes[0]
    cells = voxelize_points(points, fine)
    if not cells:
        return []
    if len(cells) > MAX_FINE_CELLS:
        cells = cap_voxels(cells, MAX_FINE_CELLS)
    cells = _dilate_cells(cells)
    cubes: list[SizedCube] = []
    remaining = cells
    for size in sorted(sizes, reverse=True):
        factor = max(int(round(size / fine)), 1)
        merged, remaining = _extract_aligned(
            remaining, fine_mm=fine, factor=factor, size_mm=size
        )
        cubes.extend(merged)
    cubes.sort(key=lambda cube: (-cube.size_mm, cube.center))
    if len(cubes) > max_cubes:
        return cubes[:max_cubes]
    return cubes


def voxels_along_segment(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    size_mm: float = BLOCK_SIZE_MM,
) -> set[VoxelKey]:
    cells = {voxel_key(start, size_mm), voxel_key(end, size_mm)}
    span = distance_mm(start, end)
    steps = max(int(span / size_mm), 1)
    for i in range(1, steps):
        t = i / steps
        cells.add(
            voxel_key(
                (
                    start[0] + (end[0] - start[0]) * t,
                    start[1] + (end[1] - start[1]) * t,
                    start[2] + (end[2] - start[2]) * t,
                ),
                size_mm,
            )
        )
    return cells


def swept_trail_centers(
    previous: list[tuple[float, float, float]] | None,
    current: list[tuple[float, float, float]],
    size_mm: float = BLOCK_SIZE_MM,
) -> list[tuple[float, float, float]]:
    """Current occupancy plus the cells crossed since ``previous`` (same-length zip)."""
    cells = voxelize_points(current, size_mm)
    if previous and len(previous) == len(current):
        for start, end in zip(previous, current):
            if distance_mm(start, end) < 1e-6:
                continue
            cells |= voxels_along_segment(start, end, size_mm)
    return centers_from_voxels(cells, size_mm)


def cubes_vertices_sized(
    cubes: list[SizedCube], *, draw_scale: float = LIVE_DRAW_SCALE
) -> list[tuple[float, float, float]]:
    vertices: list[tuple[float, float, float]] = []
    for cube in cubes:
        vertices.extend(cube_triangles(cube.center, cube.size_mm * draw_scale))
    return vertices


@dataclass
class VoxelTrail:
    """World-grid occupancy: each cell is painted at most once (the swept volume)."""

    size_mm: float = BLOCK_SIZE_MM
    max_blocks: int = MAX_BLOCKS_PER_ARM
    seen: set[VoxelKey] = field(default_factory=set)
    pending: list[tuple[float, float, float]] = field(default_factory=list)
    count: int = 0

    @property
    def pending_cubes(self) -> int:
        return len(self.pending) // VERTICES_PER_CUBE

    def absorb(
        self, centers: list[tuple[float, float, float]]
    ) -> list[tuple[float, float, float]]:
        """Queue cubes for newly occupied cells. Returns those world centres."""
        added: list[tuple[float, float, float]] = []
        for xyz in centers:
            if self.count >= self.max_blocks:
                break
            key = voxel_key(xyz, self.size_mm)
            if key in self.seen:
                continue
            self.seen.add(key)
            center = voxel_center(key, self.size_mm)
            self.pending.extend(cube_triangles(center, CUBE_DRAW_MM))
            self.count += 1
            added.append(center)
        return added

    def take_pending(self) -> list[tuple[float, float, float]]:
        vertices = self.pending
        self.pending = []
        return vertices


@dataclass
class BlockBuffer:
    """Per-arm accumulator: downsample by travel, cap total cubes, batch pending vertices."""

    sample_mm: float = SAMPLE_MM
    max_blocks: int = MAX_BLOCKS_PER_ARM
    block_size_mm: float = BLOCK_SIZE_MM
    _last: dict[str, tuple[float, float, float]] = field(default_factory=dict)
    pending: list[tuple[float, float, float]] = field(default_factory=list)
    count: int = 0

    @property
    def pending_cubes(self) -> int:
        return len(self.pending) // VERTICES_PER_CUBE

    def consider(self, point: LinkPoint) -> bool:
        """Queue a cube at ``point`` if it has travelled far enough and the cap allows."""
        if self.count >= self.max_blocks:
            return False
        last = self._last.get(point.key)
        if last is not None and distance_mm(last, point.xyz) < self.sample_mm:
            return False
        self._last[point.key] = point.xyz
        self.pending.extend(cube_triangles(point.xyz, self.block_size_mm))
        self.count += 1
        return True

    def would_add(self, point: LinkPoint) -> bool:
        """True if ``consider`` would queue a cube, without mutating the buffer."""
        if self.count >= self.max_blocks:
            return False
        last = self._last.get(point.key)
        if last is not None and distance_mm(last, point.xyz) < self.sample_mm:
            return False
        return True

    def take_pending(self) -> list[tuple[float, float, float]]:
        vertices = self.pending
        self.pending = []
        return vertices
