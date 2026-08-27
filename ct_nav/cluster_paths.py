"""Locate a ct_config cluster checkout without a hardcoded Windows drive."""

from __future__ import annotations

import os
import platform
from collections.abc import Mapping
from pathlib import Path

CLUSTER_LEAF = Path("ct_config") / "azula1"
WINDOWS_FALLBACK = Path(r"D:\Bitbucket\ct_config\azula1")


def _find_repo_root(start: Path) -> Path | None:
    for parent in [start, *start.parents]:
        if (parent / "scripts" / "install_app.py").is_file() and (parent / "ct_nav").is_dir():
            return parent
    return None


def default_repo_root() -> Path | None:
    starts = [Path.cwd()]
    here = Path(__file__).resolve().parent.parent
    if here not in starts:
        starts.append(here)
    for start in starts:
        found = _find_repo_root(start)
        if found is not None:
            return found
    return None


def cluster_candidates(
    *,
    environ: Mapping[str, str] | None = None,
    home: Path | None = None,
    repo_root: Path | None = None,
    system: str | None = None,
) -> list[Path]:
    """Ordered probe list; entries need not exist."""
    env = os.environ if environ is None else environ
    home_path = Path.home() if home is None else Path(home)
    os_name = platform.system() if system is None else system
    root = default_repo_root() if repo_root is None else Path(repo_root)

    candidates: list[Path] = []
    configured = env.get("CT_CONFIG_CLUSTER")
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(home_path / "Bitbucket" / CLUSTER_LEAF)
    candidates.append(home_path / "bitbucket" / CLUSTER_LEAF)
    if root is not None:
        candidates.append(root.parent / CLUSTER_LEAF)
    if os_name == "Windows":
        candidates.append(WINDOWS_FALLBACK)
    return candidates


def discover_cluster_dir(
    *,
    environ: Mapping[str, str] | None = None,
    home: Path | None = None,
    repo_root: Path | None = None,
    system: str | None = None,
) -> Path | None:
    """Return the first existing azula1-style cluster directory, or None.

    Probe order: ``CT_CONFIG_CLUSTER``, ``~/Bitbucket/ct_config/azula1``,
    ``~/bitbucket/ct_config/azula1``, ``<repo>/../ct_config/azula1``, then
    ``D:\\Bitbucket\\ct_config\\azula1`` on Windows only.
    """
    seen: set[Path] = set()
    for raw in cluster_candidates(
        environ=environ, home=home, repo_root=repo_root, system=system
    ):
        try:
            resolved = raw.resolve()
        except OSError:
            continue
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_dir():
            return resolved
    return None
