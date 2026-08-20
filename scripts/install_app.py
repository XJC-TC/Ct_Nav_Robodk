"""Cross-platform installer / sync for the CtNav RoboDK App.

Builds a self-contained copy of the App (see build_package.py), installs it into this
machine's RoboDK Apps folder (and, on Windows, the Add-ins folder + the AddinManager
enable list), optionally pip-installs runtime deps, and prints the Python-interpreter
path to paste into RoboDK Options.

The git checkout under `roboapp/` is never modified -- only `dist/` (gitignored) and the
local RoboDK install / user settings are touched. Re-run with `-y` / `--sync` after
changing code to refresh the installed copy.

Usage:
    python scripts/install_app.py
    python scripts/install_app.py -y --python H:\\Python3.11\\python.exe
    python scripts/install_app.py --sync          # same as -y: rebuild + overwrite
    python scripts/install_app.py --apps-dir "D:\\RoboDK\\Apps"
    python scripts/install_app.py --write-python-setting   # RoboDK must be closed
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_package  # noqa: E402

APP_NAME = "CtNav"
ADDIN_ID = "com.multiplylabs.app.ctnav"
RUNTIME_DEPS = ["robodk>=5.6.0", "PyYAML>=6.0", "PySide6>=6.5"]

# Imports that must all succeed, including a real Qt DLL load, for an interpreter to be
# usable by the App.
_DEP_CHECK = (
    "import importlib.util as u, sys\n"
    "missing=[n for n in ('robodk','yaml','PySide6') if u.find_spec(n) is None]\n"
    "if missing:\n"
    "    sys.exit(1)\n"
    "from PySide6 import QtCore  # noqa: F401\n"
)


# ---------------------------------------------------------------------------
# RoboDK path discovery
# ---------------------------------------------------------------------------

def candidate_robodk_bases() -> list[Path]:
    system = platform.system()
    home = Path.home()
    bases: list[Path] = []

    env = os.environ.get("ROBODK_HOME") or os.environ.get("ROBODK_PATH")
    if env:
        bases.append(Path(env))

    if system == "Windows":
        bases.extend(
            [
                Path("C:/RoboDK"),
                Path("D:/RoboDK"),
                Path("E:/RoboDK"),
                Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "RoboDK",
                Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
                / "RoboDK",
                home / "RoboDK",
            ]
        )
    elif system == "Darwin":
        bases.extend(
            [
                home / "RoboDK" / "RoboDK.app" / "Contents",
                home / "Applications" / "RoboDK.app" / "Contents",
                Path("/Applications/RoboDK.app/Contents"),
            ]
        )
    else:
        bases.extend([home / "RoboDK", Path("/opt/RoboDK"), Path("/usr/local/RoboDK")])

    seen: set[Path] = set()
    out: list[Path] = []
    for base in bases:
        try:
            resolved = base.expanduser().resolve()
        except OSError:
            continue
        if resolved in seen or not resolved.is_dir():
            continue
        seen.add(resolved)
        out.append(resolved)
    return out


def find_apps_dir() -> Path | None:
    for base in candidate_robodk_bases():
        apps = base / "Apps"
        if apps.is_dir():
            return apps
    return None


def find_addins_dir(apps_dir: Path) -> Path | None:
    """Sibling Addins folder next to Apps (Windows / modern RoboDK layouts)."""
    addins = apps_dir.parent / "Addins"
    return addins if addins.is_dir() else None


def prompt_for_apps_dir() -> Path:
    print("Could not auto-detect your RoboDK Apps folder.")
    print("Typical locations:")
    print("  Windows: C:\\RoboDK\\Apps  or  D:\\RoboDK\\Apps")
    print("  macOS:   <RoboDK.app>/Contents/Apps")
    print("  Linux:   ~/RoboDK/Apps")
    print("Tip: set ROBODK_HOME to your RoboDK install root to skip this prompt.")
    apps_dir = Path(input("Enter the full path to your RoboDK Apps folder: ").strip())
    if not apps_dir.expanduser().is_dir():
        raise SystemExit(f"Not a directory: {apps_dir}")
    return apps_dir.expanduser()


# ---------------------------------------------------------------------------
# Install into Apps / Addins
# ---------------------------------------------------------------------------

def _safe_rmtree(path: Path) -> None:
    """Remove a directory, junction, or file without following junctions into targets."""
    if not path.exists() and not path.is_symlink():
        return
    if path.is_symlink() or path.is_file():
        path.unlink()
        return
    try:
        if path.is_junction():  # type: ignore[attr-defined]
            path.rmdir()
            return
    except AttributeError:
        pass
    if platform.system() == "Windows":
        # Older Python has no Path.is_junction; detect the reparse point directly so
        # shutil.rmtree cannot recurse into whatever the junction points at.
        try:
            import ctypes

            FILE_ATTRIBUTE_REPARSE_POINT = 0x400
            attrs = ctypes.windll.kernel32.GetFileAttributesW(str(path))  # type: ignore[attr-defined]
            if attrs != -1 and (attrs & FILE_ATTRIBUTE_REPARSE_POINT):
                path.rmdir()
                return
        except Exception:
            pass
    shutil.rmtree(path)


def install_into_apps(apps_dir: Path, build_dir: Path, assume_yes: bool) -> Path:
    target = apps_dir / APP_NAME
    if target.exists() or target.is_symlink():
        if not assume_yes:
            reply = input(f"{target} already exists. Overwrite? [y/N] ").strip().lower()
            if reply != "y":
                raise SystemExit("Aborted, existing install left untouched.")
        _safe_rmtree(target)

    shutil.copytree(build_dir, target)
    return target


def _swap_dir(dest: Path, source: Path) -> None:
    """Put ``source`` at ``dest`` by renaming, never by deleting ``dest`` first.

    Deleting first is what emptied the Add-in while RoboDK still had files locked:
    the toolbar then pointed at an empty folder and the icon disappeared. If the live
    folder cannot be renamed aside, leave it untouched and tell the caller to quit
    RoboDK.
    """
    backup = dest.with_name(dest.name + ".replacing")
    _safe_rmtree(backup)
    if dest.exists() or dest.is_symlink():
        try:
            dest.rename(backup)
        except OSError as exc:
            raise SystemExit(
                f"Cannot replace {dest} while it is in use ({exc}).\n"
                "Quit RoboDK completely, then re-run: python scripts/install_app.py --sync"
            ) from exc
    try:
        source.rename(dest)
    except OSError as exc:
        if backup.exists() or backup.is_symlink():
            try:
                backup.rename(dest)
            except OSError:
                pass
        raise SystemExit(
            f"Cannot move {source} into place ({exc}). The previous install was restored."
        ) from exc
    _safe_rmtree(backup)


def install_into_addins(addins_dir: Path, build_dir: Path) -> Path:
    """Install as a modern Add-in: Addins/<id>/manifest.xml + CtNav/."""
    addin_root = addins_dir / ADDIN_ID
    staging = addins_dir / f".{ADDIN_ID}.staging"
    _safe_rmtree(staging)
    staging.mkdir(parents=True)

    manifest = build_dir / "manifest.xml"
    if manifest.is_file():
        shutil.copy2(manifest, staging / "manifest.xml")
    shutil.copytree(build_dir, staging / APP_NAME)

    if not (staging / APP_NAME / "AppConfig.ini").is_file() or not (
        staging / APP_NAME / f"{APP_NAME}Panel.svg"
    ).is_file():
        _safe_rmtree(staging)
        raise SystemExit(f"Staged add-in is incomplete: {staging / APP_NAME}")

    _swap_dir(addin_root, staging)
    return addin_root


def enable_addin_windows(addin_root: Path) -> None:
    """Append the add-in to the user-local AddinManager.ini Enabled list."""
    ami = Path.home() / "AppData/Roaming/RoboDK/AddinManager.ini"
    if not ami.is_file():
        print(f"Note: {ami} not found; enable CtNav in Tools > Add-in Manager after restart.")
        return

    addin_path = str(addin_root.resolve()).replace("\\", "/")
    text = ami.read_text(encoding="utf-8", errors="replace")
    if addin_path in text:
        print(f"Add-in already listed in {ami.name}")
        return

    match = re.search(r"(?ms)^\[Enabled\]\s*\nsize=(\d+)(.*?)(?=^\[|\Z)", text)
    if not match:
        print(f"Note: could not parse [Enabled] in {ami}; enable manually in Add-in Manager.")
        return

    old_size = int(match.group(1))
    body = match.group(2).rstrip("\n")
    new_enabled = f"[Enabled]\nsize={old_size + 1}{body}\n{old_size}={addin_path}\n\n"
    text = text[: match.start()] + new_enabled + text[match.end() :]

    watchdog = re.search(r"(?ms)^\[Watchdog\]\s*\nsize=(\d+)(.*?)(?=^\[|\Z)", text)
    if watchdog:
        w_size = int(watchdog.group(1))
        w_body = watchdog.group(2).rstrip("\n")
        new_watch = (
            f"[Watchdog]\nsize={w_size + 1}{w_body}\n"
            f"{w_size + 1}\\path={addin_path}\n{w_size + 1}\\checked=true\n"
        )
        text = text[: watchdog.start()] + new_watch + text[watchdog.end() :]

    ami.write_text(text, encoding="utf-8", newline="\n")
    print(f"Enabled add-in in {ami}")


# ---------------------------------------------------------------------------
# Python interpreter helpers
# ---------------------------------------------------------------------------

def _run_probe(python_path: str, code: str) -> int | None:
    try:
        result = subprocess.run(
            [python_path, "-c", code], capture_output=True, text=True, timeout=90
        )
        return result.returncode
    except (OSError, subprocess.SubprocessError):
        return None


def python_has_deps(python_path: str) -> bool:
    """True only if robodk/PyYAML/PySide6 import, including a real QtCore DLL load.

    ``find_spec('PySide6')`` alone is not enough: Anaconda base often has PySide6
    installed via pip but fails at ``from PySide6 import QtCore`` because conda Qt5 DLLs
    conflict ("DLL load failed ... specified procedure could not be found").
    """
    return _run_probe(python_path, _DEP_CHECK) == 0


def python_pyside_broken(python_path: str) -> bool:
    """True when PySide6 is present but QtCore fails to load (the conda Qt clash)."""
    code = (
        "import importlib.util as u, sys\n"
        "if u.find_spec('PySide6') is None:\n"
        "    sys.exit(2)\n"
        "try:\n"
        "    from PySide6 import QtCore  # noqa: F401\n"
        "except Exception:\n"
        "    sys.exit(1)\n"
        "sys.exit(0)\n"
    )
    return _run_probe(python_path, code) == 1


def candidate_python_interpreters() -> list[Path]:
    """Ordered search list: prefer clean CPython over conda base on Windows."""
    home = Path.home()
    candidates: list[Path] = []
    if platform.system() == "Windows":
        for drive in ("H:", "C:", "D:"):
            candidates.append(Path(f"{drive}/Python3.11/python.exe"))
            candidates.append(Path(f"{drive}/Python311/python.exe"))
            candidates.append(Path(f"{drive}/Python3.12/python.exe"))
            candidates.append(Path(f"{drive}/Python312/python.exe"))
        candidates.extend(
            [
                home / "AppData/Local/Programs/Python/Python311/python.exe",
                home / "AppData/Local/Programs/Python/Python312/python.exe",
                home / "AppData/Local/Programs/Python/Python313/python.exe",
            ]
        )
        for base in candidate_robodk_bases():
            candidates.append(base / "Python-Embedded" / "python.exe")
            candidates.append(base / "Python" / "python.exe")
        # Conda last: often has PySide6 on disk but DLL-broken against qt-main/pyqt.
        candidates.extend(
            [
                Path("H:/anaconda3/python.exe"),
                home / "anaconda3" / "python.exe",
                home / "miniconda3" / "python.exe",
                Path(os.environ.get("LOCALAPPDATA", "")) / "anaconda3" / "python.exe",
                Path(os.environ.get("LOCALAPPDATA", "")) / "miniconda3" / "python.exe",
            ]
        )
    else:
        candidates.extend(
            [
                Path("/usr/local/bin/python3"),
                Path("/usr/bin/python3"),
                Path("/opt/anaconda3/bin/python3"),
                home / "miniconda3/bin/python3",
                home / "anaconda3/bin/python3",
            ]
        )
    return candidates


def resolve_python(explicit: str | None) -> str | None:
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_file():
            raise SystemExit(f"Python not found: {explicit}")
        resolved = str(path.resolve())
        if python_pyside_broken(resolved):
            print(
                f"WARNING: {resolved} has PySide6 but QtCore fails to load "
                "(common with Anaconda base + conda qt-main/pyqt).\n"
                "  Prefer a standalone CPython (e.g. Python 3.11) for RoboDK, or a fresh "
                "conda env with only pip-installed PySide6 (no conda qt/pyqt)."
            )
        return resolved

    current = str(Path(sys.executable).resolve())
    if python_has_deps(current):
        return current
    if python_pyside_broken(current):
        print(
            f"Note: {current} cannot load PySide6.QtCore (DLL conflict). "
            "Searching for another interpreter ..."
        )

    for candidate in candidate_python_interpreters():
        if not candidate.is_file():
            continue
        path = str(candidate.resolve())
        if path != current and python_has_deps(path):
            return path

    return current if Path(current).is_file() else None


def install_dependencies(python_path: str) -> None:
    print(f"Installing runtime dependencies into {python_path} ...")
    subprocess.run([python_path, "-m", "pip", "install", *RUNTIME_DEPS], check=True)


def robodk_seems_running() -> bool:
    system = platform.system()
    try:
        if system == "Windows":
            result = subprocess.run(
                ["tasklist", "/FI", "IMAGENAME eq RoboDK.exe"],
                capture_output=True,
                text=True,
                timeout=15,
            )
            return "RoboDK.exe" in (result.stdout or "")
        args = ["pgrep", "-x", "RoboDK"] if system == "Darwin" else ["pgrep", "-f", "RoboDK"]
        return subprocess.run(args, capture_output=True, timeout=15).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def write_python_setting(python_path: str) -> None:
    """Best-effort write of Path_PythonRun into the user settings.ini (Windows).

    RoboDK exposes no stable scriptable way to set this. On Windows, writing settings.ini
    while RoboDK is closed usually works -- still verify in Tools > Options > Python.
    """
    if platform.system() != "Windows":
        print("Note: --write-python-setting is only implemented for Windows settings.ini.")
        print("Set Tools > Options > Python manually on this OS.")
        return

    if robodk_seems_running():
        raise SystemExit(
            "RoboDK appears to be running. Quit RoboDK completely, then re-run with "
            "--write-python-setting (otherwise the setting is overwritten on exit)."
        )

    settings = Path.home() / "AppData/Roaming/RoboDK/settings.ini"
    if not settings.is_file():
        raise SystemExit(f"RoboDK settings not found: {settings}")

    value = str(Path(python_path).resolve()).replace("\\", "/")
    text = settings.read_text(encoding="utf-8", errors="replace")

    # Which key holds the interpreter varies by RoboDK version: older builds use
    # Path_PythonRun, newer ones Path_PythonRun2, and a given settings.ini normally has
    # only one of them. Update every key that is present rather than betting on one.
    written: list[str] = []
    for key in ("Path_PythonRun", "Path_PythonRun2"):
        if re.search(rf"(?m)^{key}=", text):
            text = re.sub(rf"(?m)^{key}=.*$", f"{key}={value}", text)
            written.append(key)

    if not written:
        text = text.rstrip() + f"\nPath_PythonRun2={value}\n"
        written.append("Path_PythonRun2 (added)")

    settings.write_text(text, encoding="utf-8", newline="\n")
    print(f"Wrote {', '.join(written)} = {value} into {settings}")
    print("Open Tools > Options > Python once after restart to confirm it stuck.")


def print_python_path_instructions(python_path: str | None) -> None:
    print()
    print("Python interpreter for RoboDK (required):")
    print("  1. Open RoboDK")
    print("  2. Tools > Options > Python")
    print("  3. Paste this ABSOLUTE path into 'Python interpreter' (type it; the")
    print("     Select... picker only offers bare command names, which will not work):")
    if python_path:
        print(f"     {python_path}")
        if not python_has_deps(python_path):
            print("     WARNING: this interpreter is missing robodk / PyYAML / PySide6.")
            print("     Re-run: python scripts/install_app.py -y --python <path>")
    else:
        print("     <absolute path to a python with robodk, PyYAML and PySide6>")
    print("  4. Restart RoboDK")
    print()
    print("Optional, with RoboDK fully quit:")
    print("  python scripts/install_app.py -y --write-python-setting --python <path>")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--apps-dir", type=Path, help="Override auto-detected RoboDK Apps folder")
    parser.add_argument(
        "--python",
        help="Python interpreter for deps / RoboDK Options (default: this one if usable)",
    )
    parser.add_argument(
        "-y", "--yes", action="store_true", help="Overwrite existing install without asking"
    )
    parser.add_argument(
        "--sync", action="store_true", help="Rebuild and overwrite the install (implies -y)"
    )
    parser.add_argument(
        "--skip-addin", action="store_true", help="Do not install/enable the Add-ins copy"
    )
    parser.add_argument(
        "--skip-deps", action="store_true", help="Do not pip-install runtime dependencies"
    )
    parser.add_argument(
        "--write-python-setting",
        action="store_true",
        help="Best-effort write Path_PythonRun into Windows settings.ini (quit RoboDK first)",
    )
    args = parser.parse_args()

    assume_yes = args.yes or args.sync
    if args.sync:
        print("Syncing CtNav from this repo into the local RoboDK install ...")

    build_dir = build_package.build()
    print(f"Built package at: {build_dir}")

    apps_dir = (args.apps_dir or find_apps_dir() or prompt_for_apps_dir()).expanduser()
    apps_dir.mkdir(parents=True, exist_ok=True)

    print(f"Installed CtNav (AppLoader) to: {install_into_apps(apps_dir, build_dir, assume_yes)}")

    addins_dir = find_addins_dir(apps_dir)
    if addins_dir and not args.skip_addin:
        if robodk_seems_running():
            print(
                "RoboDK is running: left the Add-in folder untouched "
                "(replacing it while locked is what makes the toolbar icon vanish)."
            )
            print("Quit RoboDK, then re-run: python scripts/install_app.py --sync")
        else:
            addin_root = install_into_addins(addins_dir, build_dir)
            print(f"Installed CtNav (Add-in) to: {addin_root}")
            if platform.system() == "Windows":
                enable_addin_windows(addin_root)
    elif not args.skip_addin:
        print("Note: no Addins folder next to Apps; skipped the modern Add-in install.")

    python_path = resolve_python(args.python)
    if python_path and python_has_deps(python_path):
        print(f"Python deps OK: {python_path}")
    elif python_path and not args.skip_deps:
        install_dependencies(python_path)
    elif python_path:
        print(f"Python selected but deps incomplete: {python_path}")

    if args.write_python_setting:
        if not python_path:
            raise SystemExit("--write-python-setting requires a resolvable --python path")
        write_python_setting(python_path)

    print_python_path_instructions(python_path)
    print("Done. Restart RoboDK to reload the toolbar / Add-in.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
