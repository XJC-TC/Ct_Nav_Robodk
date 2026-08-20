"""Discover and swap the EOATs modelled under one MHR in a RoboDK station.

Each arm typically carries every EOAT as a child Tool (or CAD object) of the robot, with
visibility used as the swap. ``CABLE GUARD`` is not an EOAT in that sense: it stays on
the flange for every tool, so a swap hides every other EOAT and leaves the cable guard
visible.

Identification is by walking the robot's station-tree children (and any Tool items
RoboDK reports as linked to it). Names are matched loosely so ``CABLE GUARD``,
``Cable_Guard`` and ``MHR-XZ CABLE GUARD`` all count as the keep-visible piece.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from robodk import robolink

# Folders / default TCPs that sit under a UR but are not EOATs.
_SKIP_COMPACT = frozenset(
    {
        "tcp",
        "flange",
        "tool",
        "tool1",
        "defaulttool",
        "ur10ebase",
        "ur10e",
        "ur12e",
        "ur16e",
    }
)

_MAX_WALK_DEPTH = 6


class EoatError(Exception):
    """Raised when an EOAT cannot be found or applied on the station."""


def compact_name(name: str) -> str:
    return "".join(ch for ch in name.lower() if ch.isalnum())


def is_cable_guard(name: str) -> bool:
    """True for the flange cable-guard CAD that must stay visible through every swap."""
    return "cableguard" in compact_name(name)


def is_swappable_eoat_name(name: str, item_type: int) -> bool:
    """True when ``name`` looks like an EOAT that should be hidden when another is fitted."""
    if is_cable_guard(name):
        return False
    compact = compact_name(name)
    if compact in _SKIP_COMPACT:
        return False
    if item_type == robolink.ITEM_TYPE_TOOL:
        return True
    lowered = name.lower()
    return "eoat" in lowered or "end of arm" in lowered


@dataclass(frozen=True)
class EoatItem:
    name: str
    kind: str  # "tool" | "object" | "frame"
    keep_visible: bool
    visible: bool


@dataclass
class EoatInventory:
    """The EOATs found under one arm, split into swappable vs always-on."""

    arm: str
    robot_item: str
    swappable: list[EoatItem] = field(default_factory=list)
    keep_visible: list[EoatItem] = field(default_factory=list)

    @property
    def active(self) -> str | None:
        """The swappable EOAT that is currently visible, if exactly one is."""
        shown = [item.name for item in self.swappable if item.visible]
        return shown[0] if len(shown) == 1 else None

    def names(self) -> list[str]:
        return [item.name for item in self.swappable]


def _type_kind(item_type: int) -> str:
    if item_type == robolink.ITEM_TYPE_TOOL:
        return "tool"
    if item_type == robolink.ITEM_TYPE_FRAME:
        return "frame"
    return "object"


def _visible(item: robolink.Item) -> bool:
    try:
        return bool(item.Visible())
    except Exception:
        return False


def _ancestors_include(item: robolink.Item, robot: robolink.Item) -> bool:
    current = item
    robot_name = robot.Name()
    for _ in range(32):
        try:
            current = current.Parent()
        except Exception:
            return False
        if not current.Valid():
            return False
        if current == robot or current.Name() == robot_name:
            return True
        if current.Type() == robolink.ITEM_TYPE_STATION:
            return False
    return False


def _walk_children(root: robolink.Item, *, max_depth: int = _MAX_WALK_DEPTH):
    """Yield descendants, without descending into another robot."""

    def walk(item: robolink.Item, depth: int):
        if depth > max_depth:
            return
        try:
            children = item.Childs()
        except Exception:
            return
        for child in children:
            yield child
            if child.Type() in (robolink.ITEM_TYPE_ROBOT, robolink.ITEM_TYPE_ROBOT_ARM):
                continue
            yield from walk(child, depth + 1)

    yield from walk(root, 0)


def _collect_candidates(rdk: robolink.Robolink, robot: robolink.Item) -> list[robolink.Item]:
    found: dict[str, robolink.Item] = {}

    def consider(item: robolink.Item) -> None:
        if not item.Valid():
            return
        name = item.Name()
        if name in found:
            return
        item_type = item.Type()
        if is_cable_guard(name) or is_swappable_eoat_name(name, item_type):
            found[name] = item

    for child in _walk_children(robot):
        consider(child)

    try:
        for tool in robot.getLinks(robolink.ITEM_TYPE_TOOL) or []:
            consider(tool)
    except Exception:
        pass

    for tool in rdk.ItemList(robolink.ITEM_TYPE_TOOL):
        try:
            if _ancestors_include(tool, robot):
                consider(tool)
        except Exception:
            continue

    return list(found.values())


def list_eoats(rdk: robolink.Robolink, robot: robolink.Item, arm_name: str) -> EoatInventory:
    """EOATs parented under ``robot``, plus any Tool RoboDK lists as belonging to it."""
    inventory = EoatInventory(arm=arm_name, robot_item=robot.Name())
    for item in _collect_candidates(rdk, robot):
        record = EoatItem(
            name=item.Name(),
            kind=_type_kind(item.Type()),
            keep_visible=is_cable_guard(item.Name()),
            visible=_visible(item),
        )
        if record.keep_visible:
            inventory.keep_visible.append(record)
        else:
            inventory.swappable.append(record)
    inventory.swappable.sort(key=lambda e: e.name.lower())
    inventory.keep_visible.sort(key=lambda e: e.name.lower())
    return inventory


def _item_by_name(candidates: list[robolink.Item], name: str) -> robolink.Item | None:
    for item in candidates:
        if item.Name() == name:
            return item
    return None


def _set_tree_visible(root: robolink.Item, visible: bool) -> None:
    """Show or hide ``root`` and its descendants, never hiding a cable guard."""
    want = 1 if visible else 0
    try:
        if is_cable_guard(root.Name()):
            root.setVisible(1)
        else:
            root.setVisible(want)
    except Exception:
        return
    for child in _walk_children(root):
        try:
            if is_cable_guard(child.Name()):
                child.setVisible(1)
            elif visible:
                child.setVisible(1)
        except Exception:
            continue


def apply_eoat(
    rdk: robolink.Robolink,
    robot: robolink.Item,
    name: str | None,
) -> EoatInventory:
    """Fit ``name`` on ``robot``: hide every other EOAT, leave cable guards visible.

    ``name is None`` means a bare flange: all swappable EOATs hidden, cable guard on.
    The selected item is also made the robot's active tool when it is a Tool, so the
    TCP matches the CAD that is showing.
    """
    candidates = _collect_candidates(rdk, robot)
    swappable = [item for item in candidates if not is_cable_guard(item.Name())]
    guards = [item for item in candidates if is_cable_guard(item.Name())]

    chosen: robolink.Item | None = None
    if name:
        chosen = _item_by_name(swappable, name)
        if chosen is None:
            have = ", ".join(item.Name() for item in swappable) or "none"
            raise EoatError(
                f"{robot.Name()}: no EOAT named {name!r} (have: {have})"
            )

    for item in swappable:
        _set_tree_visible(item, False)
    if chosen is not None:
        _set_tree_visible(chosen, True)
        for item in swappable:
            if item.Name() != chosen.Name():
                try:
                    item.setVisible(0)
                except Exception:
                    pass
    for item in guards:
        _set_tree_visible(item, True)

    if chosen is not None and chosen.Type() == robolink.ITEM_TYPE_TOOL:
        robot.setPoseTool(chosen)

    return list_eoats(rdk, robot, robot.Name())
