"""Cross-platform installer / sync for the CtNav RoboDK App.

Builds a self-contained copy of the App (see build_package.py) and installs it:

- Windows: RoboDK install-root ``Apps/`` (AppLoader) plus ``Addins/``, and Enable in
  ``AddinManager.ini`` when that file already exists.
- Linux / macOS: user-level Add-ins only (never the ``.app`` bundle). Creates the
  Addins directory if needed. Does not invent ``settings.ini`` / ``AddinManager.ini``.

On Linux / macOS, runtime deps go in ``{repo}/.venv`` created with this interpreter
(Python 3.9+). Windows keeps the existing interpreter probe and does not create a venv.

``-y`` / ``--sync`` writes the interpreter absolute path into existing RoboDK
``settings.ini`` files when RoboDK is fully quit.

Usage:
    python3 scripts/install_app.py -y          # Linux / macOS (quit RoboDK first)
    python scripts/install_app.py -y           # Windows
    python scripts/install_app.py --sync
    python scripts/install_app.py --python /path/to/python
    python scripts/install_app.py --apps-dir "D:\\RoboDK\\Apps"
    python scripts/install_app.py --addins-dir "~/Library/Application Support/RoboDK/Addins"
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_package  # noqa: E402

APP_NAME = "CtNav"
ADDIN_ID = "com.multiplylabs.app.ctnav"
RUNTIME_DEPS = ["robodk>=5.6.0", "PyYAML>=6.0", "PySide6>=6.5"]
MIN_PYTHON = (3, 9)
REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = REPO_ROOT / ".venv"

# Imports that must all succeed, including a real Qt DLL load, for an interpreter to be
# usable by the App.
_DEP_CHECK = (
    "import importlib.util as u, sys\n"
    "missing=[n for n in ('robodk','yaml','PySide6') if u.find_spec(n) is None]\n"
    "if missing:\n"
    "    sys.exit(1)\n"
    "from PySide6 import QtCore  # noqa: F401\n"
)

_OPEN_ONCE = (
    "RoboDK has not written its user settings yet. Open RoboDK once, quit it fully, "
    "then re-run:\n  python3 scripts/install_app.py -y"
)

_VENV_FAIL = (
    "Could not create a venv with {creator}.\n"
    "On Debian/Ubuntu: sudo apt install python3-venv\n"
    "On macOS: do not use the Apple /usr/bin/python3 stub; install Python 3.9+ from "
    "python.org or Homebrew, then re-run:\n"
    "  python3 scripts/install_app.py -y"
)

_PYSIDE_LINUX = (
    "PySide6 is installed but QtCore failed to load (missing system libraries).\n"
    "Debian/Ubuntu: sudo apt install libxcb-cursor0 libxkbcommon-x11-0 libxcb-icccm4 "
    "libxcb-keysyms1 libxcb-randr0 libxcb-render-util0\n"
    "Fedora:        sudo dnf install xcb-util-cursor libxkbcommon-x11 xcb-util-wm "
    "xcb-util-keysyms\n"
    "Then re-run: python3 scripts/install_app.py -y"
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


def robodk_user_data_dirs(
    *,
    system: str | None = None,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
) -> list[Path]:
    """Candidate roots for settings.ini, AddinManager.ini, and user Addins."""
    os_name = platform.system() if system is None else system
    home_path = Path.home() if home is None else Path(home)
    env = os.environ if environ is None else environ

    if os_name == "Windows":
        return [home_path / "AppData/Roaming/RoboDK"]
    if os_name == "Darwin":
        return [home_path / "Library/Application Support/RoboDK"]

    xdg_data = Path(env["XDG_DATA_HOME"]) if env.get("XDG_DATA_HOME") else home_path / ".local/share"
    xdg_config = Path(env["XDG_CONFIG_HOME"]) if env.get("XDG_CONFIG_HOME") else home_path / ".config"
    dirs = [xdg_data / "RoboDK", xdg_config / "RoboDK"]
    seen: set[Path] = set()
    out: list[Path] = []
    for path in dirs:
        if path in seen:
            continue
        seen.add(path)
        out.append(path)
    return out


def user_addins_dir(
    *,
    system: str | None = None,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
) -> Path:
    """Canonical user-level Addins directory (need not exist yet)."""
    os_name = platform.system() if system is None else system
    home_path = Path.home() if home is None else Path(home)
    env = os.environ if environ is None else environ
    if os_name == "Windows":
        return home_path / "AppData/Roaming/RoboDK/Addins"
    if os_name == "Darwin":
        return home_path / "Library/Application Support/RoboDK/Addins"
    xdg_data = Path(env["XDG_DATA_HOME"]) if env.get("XDG_DATA_HOME") else home_path / ".local/share"
    return xdg_data / "RoboDK" / "Addins"


def candidate_settings_ini(
    *,
    system: str | None = None,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
) -> list[Path]:
    return [
        directory / "settings.ini"
        for directory in robodk_user_data_dirs(system=system, home=home, environ=environ)
    ]


def candidate_addin_manager_ini(
    *,
    system: str | None = None,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
) -> list[Path]:
    return [
        directory / "AddinManager.ini"
        for directory in robodk_user_data_dirs(system=system, home=home, environ=environ)
    ]


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


def _patch_addin_manager_text(text: str, addin_path: str) -> str | None:
    if addin_path in text:
        return text
    match = re.search(r"(?ms)^\[Enabled\]\s*\nsize=(\d+)(.*?)(?=^\[|\Z)", text)
    if not match:
        return None

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
    return text


def enable_addin(
    addin_root: Path,
    *,
    ini_files: list[Path] | None = None,
    system: str | None = None,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
) -> str:
    """Append the add-in to existing AddinManager.ini files. Never creates the file.

    Returns ``enabled``, ``already``, ``missing``, or ``unparsed``.
    """
    files = ini_files
    if files is None:
        files = [
            path
            for path in candidate_addin_manager_ini(system=system, home=home, environ=environ)
            if path.is_file()
        ]
    existing = [path for path in files if path.is_file()]
    if not existing:
        print(
            "Note: AddinManager.ini not found; enable CtNav in Tools > Add-in Manager "
            "after restart, or open RoboDK once then re-run this installer."
        )
        return "missing"

    addin_path = str(addin_root.resolve()).replace("\\", "/")
    statuses: list[str] = []
    for ami in existing:
        text = ami.read_text(encoding="utf-8", errors="replace")
        if addin_path in text:
            print(f"Add-in already listed in {ami}")
            statuses.append("already")
            continue
        patched = _patch_addin_manager_text(text, addin_path)
        if patched is None:
            print(f"Note: could not parse [Enabled] in {ami}; enable manually in Add-in Manager.")
            statuses.append("unparsed")
            continue
        ami.write_text(patched, encoding="utf-8", newline="\n")
        print(f"Enabled add-in in {ami}")
        statuses.append("enabled")

    if "enabled" in statuses:
        return "enabled"
    if "already" in statuses:
        return "already"
    return "unparsed"


def enable_addin_windows(addin_root: Path) -> str:
    """Windows entry point used by the install-root Add-ins copy."""
    return enable_addin(addin_root, system="Windows")


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


def python_meets_min(python_path: str) -> bool:
    code = (
        "import sys\n"
        f"sys.exit(0 if sys.version_info >= {MIN_PYTHON} else 1)\n"
    )
    return _run_probe(python_path, code) == 0


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


def venv_python(venv_dir: Path, *, system: str | None = None) -> Path:
    os_name = platform.system() if system is None else system
    if os_name == "Windows":
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python"


def ensure_venv(creator: str, venv_dir: Path, *, system: str | None = None) -> str:
    """Create ``venv_dir`` with ``creator`` if needed; return the venv interpreter path."""
    python_path = venv_python(venv_dir, system=system)
    if python_path.is_file():
        return str(python_path.resolve())
    try:
        result = subprocess.run(
            [creator, "-m", "venv", str(venv_dir)],
            capture_output=True,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise SystemExit(_VENV_FAIL.format(creator=creator) + f"\n({exc})") from exc
    if result.returncode != 0 or not python_path.is_file():
        detail = (result.stderr or result.stdout or "").strip()
        extra = f"\n{detail}" if detail else ""
        raise SystemExit(_VENV_FAIL.format(creator=creator) + extra)
    return str(python_path.resolve())


def install_dependencies(python_path: str) -> None:
    print(f"Installing runtime dependencies into {python_path} ...")
    try:
        subprocess.run(
            [python_path, "-m", "pip", "install", *RUNTIME_DEPS],
            check=True,
        )
    except subprocess.CalledProcessError as exc:
        extra = ""
        if platform.system() != "Windows":
            extra = (
                "\nIf pip refused an externally-managed environment, drop --python "
                f"so CtNav can use {VENV_DIR}."
            )
        raise SystemExit(
            f"pip install failed for {python_path} (exit {exc.returncode}).{extra}"
        ) from exc


def pyside_load_failure_hint(*, system: str | None = None) -> str:
    os_name = platform.system() if system is None else system
    if os_name == "Linux":
        return _PYSIDE_LINUX
    if os_name == "Darwin":
        return (
            "PySide6 is installed but QtCore failed to load.\n"
            "Use a Homebrew or python.org CPython 3.9+, not conda base, then re-run:\n"
            "  python3 scripts/install_app.py -y"
        )
    return (
        "PySide6 is installed but QtCore failed to load "
        "(common with Anaconda base + conda qt-main/pyqt). "
        "Use a standalone CPython."
    )


def require_working_pyside(python_path: str, *, system: str | None = None) -> None:
    if python_has_deps(python_path):
        return
    if python_pyside_broken(python_path):
        raise SystemExit(pyside_load_failure_hint(system=system))
    raise SystemExit(
        f"{python_path} is missing robodk / PyYAML / PySide6 after install."
    )


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


def patch_settings_python(settings: Path, python_path: str) -> list[str]:
    """Update Path_PythonRun keys in an existing settings.ini. Does not create the file."""
    value = str(Path(python_path).resolve()).replace("\\", "/")
    text = settings.read_text(encoding="utf-8", errors="replace")
    written: list[str] = []
    for key in ("Path_PythonRun", "Path_PythonRun2"):
        if re.search(rf"(?m)^{key}=", text):
            text = re.sub(rf"(?m)^{key}=.*$", f"{key}={value}", text)
            written.append(key)
    if not written:
        text = text.rstrip() + f"\nPath_PythonRun2={value}\n"
        written.append("Path_PythonRun2 (added)")
    settings.write_text(text, encoding="utf-8", newline="\n")
    return written


def write_python_setting(
    python_path: str,
    *,
    running: bool | None = None,
    settings_files: list[Path] | None = None,
    system: str | None = None,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
) -> list[Path]:
    """Write Path_PythonRun into every existing settings.ini. Never creates the file."""
    if running is None:
        running = robodk_seems_running()
    if running:
        raise SystemExit(
            "RoboDK appears to be running. Quit RoboDK completely, then re-run:\n"
            "  python3 scripts/install_app.py -y\n"
            "(otherwise the setting is overwritten on exit)."
        )

    files = settings_files
    if files is None:
        files = [
            path
            for path in candidate_settings_ini(system=system, home=home, environ=environ)
            if path.is_file()
        ]
    existing = [path for path in files if path.is_file()]
    if not existing:
        raise SystemExit(_OPEN_ONCE)

    written_files: list[Path] = []
    for settings in existing:
        keys = patch_settings_python(settings, python_path)
        print(f"Wrote {', '.join(keys)} = {python_path} into {settings}")
        written_files.append(settings)
    print("Open Tools > Options > Python once after restart to confirm it stuck.")
    return written_files


def print_python_path_instructions(python_path: str | None, *, wrote_settings: bool) -> None:
    print()
    if wrote_settings and python_path:
        print("Python interpreter for RoboDK:")
        print(f"  {python_path}")
        print("  Confirm Tools > Options > Python after restart (must stay an absolute path).")
        return
    print("Python interpreter for RoboDK (required):")
    print("  1. Open RoboDK")
    print("  2. Tools > Options > Python")
    print("  3. Paste this ABSOLUTE path into 'Python interpreter' (type it; the")
    print("     Select... picker only offers bare command names, which will not work):")
    if python_path:
        print(f"     {python_path}")
        if not python_has_deps(python_path):
            print("     WARNING: this interpreter is missing robodk / PyYAML / PySide6.")
            print("     Re-run: python3 scripts/install_app.py -y")
    else:
        print("     <absolute path to a python with robodk, PyYAML and PySide6>")
    print("  4. Restart RoboDK")
    print()
    print("With RoboDK fully quit, this is written for you by:")
    print("  python3 scripts/install_app.py -y")


def resolve_unix_python(explicit: str | None) -> str:
    if explicit:
        resolved = resolve_python(explicit)
        if not resolved:
            raise SystemExit(f"Python not found: {explicit}")
        return resolved
    creator = str(Path(sys.executable).resolve())
    if not python_meets_min(creator):
        raise SystemExit(
            f"{creator} is older than Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]}. "
            "Install Python 3.9+ and re-run: python3 scripts/install_app.py -y"
        )
    return ensure_venv(creator, VENV_DIR)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--apps-dir", type=Path, help="Override auto-detected RoboDK Apps folder (Windows)")
    parser.add_argument(
        "--addins-dir",
        type=Path,
        help="Override user-level RoboDK Addins folder (Linux / macOS; Windows if set)",
    )
    parser.add_argument(
        "--python",
        help="Python interpreter for deps / RoboDK Options (skips .venv on Linux/macOS)",
    )
    parser.add_argument(
        "-y", "--yes", action="store_true", help="Overwrite existing install; write Python path if RoboDK is quit"
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
        help="Write Path_PythonRun into existing settings.ini (quit RoboDK first; implied by -y/--sync)",
    )
    args = parser.parse_args()

    assume_yes = args.yes or args.sync
    write_settings = assume_yes or args.write_python_setting
    windows = platform.system() == "Windows"
    if args.sync:
        print("Syncing CtNav from this repo into the local RoboDK install ...")

    build_dir = build_package.build()
    print(f"Built package at: {build_dir}")

    running = robodk_seems_running()
    problems: list[str] = []
    addin_root: Path | None = None

    if windows:
        apps_dir = (args.apps_dir or find_apps_dir() or prompt_for_apps_dir()).expanduser()
        apps_dir.mkdir(parents=True, exist_ok=True)
        print(f"Installed CtNav (AppLoader) to: {install_into_apps(apps_dir, build_dir, assume_yes)}")
        addins_dir = args.addins_dir.expanduser() if args.addins_dir else find_addins_dir(apps_dir)
    else:
        if args.apps_dir:
            print("Note: --apps-dir is ignored on Linux/macOS; CtNav installs to user Addins only.")
        addins_dir = (args.addins_dir or user_addins_dir()).expanduser()
        addins_dir.mkdir(parents=True, exist_ok=True)

    if not args.skip_addin:
        if not addins_dir:
            print("Note: no Addins folder next to Apps; skipped the modern Add-in install.")
            if write_settings:
                problems.append("No Addins folder; skipped Enable.")
        elif running:
            print(
                "RoboDK is running: left the Add-in folder untouched "
                "(replacing it while locked is what makes the toolbar icon vanish)."
            )
            print("Quit RoboDK, then re-run: python3 scripts/install_app.py -y")
            problems.append("RoboDK is running; Add-in was not replaced and settings were not written.")
        else:
            addin_root = install_into_addins(addins_dir, build_dir)
            print(f"Installed CtNav (Add-in) to: {addin_root}")
            status = enable_addin(addin_root)
            if status == "missing" and write_settings:
                problems.append(
                    "AddinManager.ini not found. Open RoboDK once, quit fully, then re-run:\n"
                    "  python3 scripts/install_app.py -y"
                )
            elif status == "unparsed" and write_settings:
                problems.append("Could not parse AddinManager.ini; enable CtNav in Tools > Add-in Manager.")

    if windows:
        python_path = resolve_python(args.python)
    else:
        python_path = resolve_unix_python(args.python)

    if python_path and python_has_deps(python_path):
        print(f"Python deps OK: {python_path}")
    elif python_path and not args.skip_deps:
        install_dependencies(python_path)
        require_working_pyside(python_path)
        print(f"Python deps OK: {python_path}")
    elif python_path:
        print(f"Python selected but deps incomplete: {python_path}")
    else:
        problems.append("No Python interpreter resolved.")

    wrote_settings = False
    if write_settings and python_path and not running:
        try:
            write_python_setting(python_path)
            wrote_settings = True
        except SystemExit as exc:
            problems.append(str(exc))
    elif write_settings and running:
        # Already recorded when the add-in was skipped; still say it if add-in was skipped via flag.
        if "RoboDK is running" not in " ".join(problems):
            problems.append(
                "RoboDK is running; settings.ini was not written. Quit, then re-run: "
                "python3 scripts/install_app.py -y"
            )

    print_python_path_instructions(python_path, wrote_settings=wrote_settings)
    if problems:
        print("Install did not finish:")
        for problem in problems:
            print(f"  {problem}")
        return 1
    print("Done. Restart RoboDK to reload the toolbar / Add-in.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
