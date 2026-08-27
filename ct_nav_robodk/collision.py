"""Our own cube-vs-entity collision, no RoboDK collision engine.

A colour block is an axis-aligned cube. An entity is either a cached station AABB
(other robots' bones, cell objects) or another arm's already-painted cubes. Overlap
is a few float compares, cheap enough to run on every playback frame.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

from .path_geom import BLOCK_SIZE_MM, LINK_RADIUS_MM, is_trace_name


class CollisionError(Exception):
    """Raised when a path/collision session cannot be set up."""


@dataclass(frozen=True)
class Aabb:
    """World-frame axis-aligned box. ``name`` is only for the status line."""

    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]
    name: str = ""

    def overlaps(self, other: "Aabb") -> bool:
        a, b = self.minimum, self.maximum
        c, d = other.minimum, other.maximum
        return not (
            b[0] < c[0]
            or a[0] > d[0]
            or b[1] < c[1]
            or a[1] > d[1]
            or b[2] < c[2]
            or a[2] > d[2]
        )


@dataclass(frozen=True)
class CollisionHit:
    cube_key: str
    entity: str

    def describe(self) -> str:
        return f"cube {self.cube_key} vs {self.entity}"


@dataclass(frozen=True)
class CollisionReport:
    hits: tuple[CollisionHit, ...] = ()

    @property
    def hit(self) -> bool:
        return bool(self.hits)

    def describe(self) -> str:
        if not self.hits:
            return "no collision"
        return "; ".join(item.describe() for item in self.hits)


# Back-compat alias used by older tests / imports.
CollisionPair = CollisionHit


def cube_aabb(
    center: tuple[float, float, float],
    size_mm: float = BLOCK_SIZE_MM,
    name: str = "",
) -> Aabb:
    half = size_mm / 2.0
    return Aabb(
        minimum=(center[0] - half, center[1] - half, center[2] - half),
        maximum=(center[0] + half, center[1] + half, center[2] + half),
        name=name,
    )


def bone_aabb(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius_mm: float = LINK_RADIUS_MM,
    name: str = "",
) -> Aabb:
    """World AABB of a capsule between two link origins, used for other robots."""
    return Aabb(
        minimum=(
            min(start[0], end[0]) - radius_mm,
            min(start[1], end[1]) - radius_mm,
            min(start[2], end[2]) - radius_mm,
        ),
        maximum=(
            max(start[0], end[0]) + radius_mm,
            max(start[1], end[1]) + radius_mm,
            max(start[2], end[2]) + radius_mm,
        ),
        name=name,
    )


def aabb_from_points(
    points: list[tuple[float, float, float]], name: str = ""
) -> Aabb | None:
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return Aabb(
        minimum=(min(xs), min(ys), min(zs)),
        maximum=(max(xs), max(ys), max(zs)),
        name=name,
    )


def first_overlap(
    cube: Aabb,
    obstacles: list[Aabb],
    skip_names: set[str] | None = None,
) -> Aabb | None:
    skip = skip_names or set()
    for obstacle in obstacles:
        if obstacle.name and obstacle.name in skip:
            continue
        if cube.overlaps(obstacle):
            return obstacle
    return None


def on_own_body(cube: Aabb, own: list[Aabb]) -> bool:
    """True when the cube is sitting on the moving arm (expected, not a collision)."""
    return first_overlap(cube, own) is not None


def keep_entity_name(name: str, *, visible: bool) -> bool:
    """Static-index filter: drop our traces and anything the station is hiding."""
    if not visible or is_trace_name(name):
        return False
    return True


class CubeOccupancy:
    """Spatial hash of already-painted cubes, so later arms can hit earlier trails.

    Cell size equals the cube size, so a query only looks at the centre cell and
    its 26 neighbours.
    """

    def __init__(self, cell_mm: float = BLOCK_SIZE_MM) -> None:
        self.cell_mm = max(cell_mm, 1.0)
        self._cells: dict[tuple[int, int, int], list[tuple[str, tuple[float, float, float]]]] = (
            defaultdict(list)
        )
        self._by_arm: dict[str, list[tuple[int, int, int]]] = defaultdict(list)

    def _key(self, center: tuple[float, float, float]) -> tuple[int, int, int]:
        return (
            int(center[0] // self.cell_mm),
            int(center[1] // self.cell_mm),
            int(center[2] // self.cell_mm),
        )

    def clear_arm(self, arm: str) -> None:
        for cell in self._by_arm.pop(arm, []):
            remaining = [entry for entry in self._cells[cell] if entry[0] != arm]
            if remaining:
                self._cells[cell] = remaining
            else:
                self._cells.pop(cell, None)

    def add(self, arm: str, center: tuple[float, float, float]) -> None:
        cell = self._key(center)
        self._cells[cell].append((arm, center))
        self._by_arm[arm].append(cell)

    def hit(
        self, cube: Aabb, ignore_arm: str, size_mm: float = BLOCK_SIZE_MM
    ) -> str | None:
        cx = (cube.minimum[0] + cube.maximum[0]) * 0.5
        cy = (cube.minimum[1] + cube.maximum[1]) * 0.5
        cz = (cube.minimum[2] + cube.maximum[2]) * 0.5
        origin = self._key((cx, cy, cz))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for arm, center in self._cells.get(
                        (origin[0] + dx, origin[1] + dy, origin[2] + dz), ()
                    ):
                        if arm == ignore_arm:
                            continue
                        if cube.overlaps(cube_aabb(center, size_mm, name=arm)):
                            return f"{arm} path cube"
        return None


@dataclass
class EntityIndex:
    """Static cell AABBs + other-robot bones + other-arm cubes."""

    static: list[Aabb] = field(default_factory=list)
    moving: list[Aabb] = field(default_factory=list)
    cubes: CubeOccupancy = field(default_factory=CubeOccupancy)

    def hit(
        self,
        cube: Aabb,
        ignore_arm: str,
        size_mm: float = BLOCK_SIZE_MM,
        skip_entities: set[str] | None = None,
    ) -> CollisionHit | None:
        obstacle = first_overlap(
            cube, self.moving, skip_entities
        ) or first_overlap(cube, self.static, skip_entities)
        if obstacle is not None:
            return CollisionHit(cube_key=cube.name, entity=obstacle.name)
        other = self.cubes.hit(cube, ignore_arm, size_mm)
        if other is not None:
            return CollisionHit(cube_key=cube.name, entity=other)
        return None
