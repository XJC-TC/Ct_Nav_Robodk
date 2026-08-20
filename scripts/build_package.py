"""Build a self-contained copy of the CtNav RoboDK App under dist/.

RoboDK runs an App's Action script with the App folder on ``sys.path`` and nothing else
from this repository, so the installed copy has to carry its own libraries. This copies
the App shell from ``roboapp/CtNav/`` and vendors ``ct_nav/``, ``ct_nav_robodk/``, the
station map and the requirements file in beside it. Nothing under ``roboapp/`` is
modified; only ``dist/`` (gitignored) is written.

A ``station_map.local.yaml`` produced by the panel's rail calibration is vendored in
preference to the tracked map, since it describes the machine the App is being installed
on.

Usage:
    python scripts/build_package.py
"""

from __future__ import annotations

import configparser
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
APP_SRC = REPO_ROOT / "roboapp" / "CtNav"
DIST_DIR = REPO_ROOT / "dist"
BUILD_DIR = DIST_DIR / "CtNav"

VENDORED_PACKAGES = ("ct_nav", "ct_nav_robodk")
VENDORED_FILES = ("requirements.txt",)

IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo")


def read_version() -> str:
    config = configparser.ConfigParser()
    config.read(APP_SRC / "AppConfig.ini")
    return config.get("General", "Version", fallback="0.0.0")


def _station_map_source() -> Path:
    local = REPO_ROOT / "station_map.local.yaml"
    return local if local.is_file() else REPO_ROOT / "station_map.yaml"


def build() -> Path:
    if not APP_SRC.is_dir():
        raise SystemExit(f"App source not found: {APP_SRC}")
    for package in VENDORED_PACKAGES:
        if not (REPO_ROOT / package / "__init__.py").is_file():
            raise SystemExit(f"Library package not found: {REPO_ROOT / package}")

    station_map = _station_map_source()
    if not station_map.is_file():
        raise SystemExit(
            f"No station map found at {station_map}. "
            "Run scripts/inspect_station.py and write one before installing."
        )

    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
    BUILD_DIR.mkdir(parents=True)

    for item in APP_SRC.iterdir():
        if item.name in {"__pycache__", *VENDORED_PACKAGES}:
            continue
        if item.is_file():
            shutil.copy2(item, BUILD_DIR / item.name)
        elif item.is_dir():
            shutil.copytree(item, BUILD_DIR / item.name, ignore=IGNORE)

    for package in VENDORED_PACKAGES:
        shutil.copytree(REPO_ROOT / package, BUILD_DIR / package, ignore=IGNORE)
    for name in VENDORED_FILES:
        source = REPO_ROOT / name
        if source.is_file():
            shutil.copy2(source, BUILD_DIR / name)

    # station_map.py resolves the default map relative to its own parent's parent, which
    # in the installed layout is the App folder itself.
    shutil.copy2(station_map, BUILD_DIR / "station_map.yaml")

    missing = [
        path
        for path in (
            BUILD_DIR / "CtNavPanel.py",
            BUILD_DIR / "AppConfig.ini",
            BUILD_DIR / "station_map.yaml",
            BUILD_DIR / "ct_nav" / "planner.py",
            BUILD_DIR / "ct_nav_robodk" / "ui" / "panel.py",
        )
        if not path.is_file()
    ]
    if missing:
        raise SystemExit("Build incomplete, missing: " + ", ".join(str(p) for p in missing))

    return BUILD_DIR


def make_zip(build_dir: Path) -> Path:
    zip_base = DIST_DIR / f"CtNav-v{read_version()}"
    return Path(
        shutil.make_archive(str(zip_base), "zip", root_dir=DIST_DIR, base_dir="CtNav")
    )


def main() -> int:
    build_dir = build()
    print(f"Built: {build_dir}")
    print(f"Zipped: {make_zip(build_dir)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
