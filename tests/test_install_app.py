"""Installer path / settings helpers. Does not need a live RoboDK."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import install_app  # noqa: E402


def test_user_addins_dir_linux_default(tmp_path: Path) -> None:
    got = install_app.user_addins_dir(system="Linux", home=tmp_path, environ={})
    assert got == tmp_path / ".local/share/RoboDK/Addins"


def test_user_addins_dir_linux_xdg_data_home(tmp_path: Path) -> None:
    xdg = tmp_path / "xdg-data"
    got = install_app.user_addins_dir(
        system="Linux",
        home=tmp_path,
        environ={"XDG_DATA_HOME": str(xdg)},
    )
    assert got == xdg / "RoboDK" / "Addins"


def test_user_addins_dir_macos(tmp_path: Path) -> None:
    got = install_app.user_addins_dir(system="Darwin", home=tmp_path, environ={})
    assert got == tmp_path / "Library/Application Support/RoboDK/Addins"


def test_linux_settings_candidates_include_config_and_data(tmp_path: Path) -> None:
    cands = install_app.candidate_settings_ini(system="Linux", home=tmp_path, environ={})
    assert cands == [
        tmp_path / ".local/share/RoboDK/settings.ini",
        tmp_path / ".config/RoboDK/settings.ini",
    ]


def test_macos_settings_candidate(tmp_path: Path) -> None:
    cands = install_app.candidate_settings_ini(system="Darwin", home=tmp_path, environ={})
    assert cands == [tmp_path / "Library/Application Support/RoboDK/settings.ini"]


def test_patch_settings_updates_existing_key(tmp_path: Path) -> None:
    ini = tmp_path / "settings.ini"
    ini.write_text("[General]\nPath_PythonRun2=/old/python3\n", encoding="utf-8")
    interp = tmp_path / "venv" / "bin" / "python3"
    interp.parent.mkdir(parents=True)
    interp.write_text("", encoding="utf-8")
    keys = install_app.patch_settings_python(ini, str(interp))
    assert keys == ["Path_PythonRun2"]
    text = ini.read_text(encoding="utf-8")
    written = str(interp.resolve()).replace("\\", "/")
    assert f"Path_PythonRun2={written}" in text
    assert "/old/python3" not in text


def test_write_python_setting_does_not_invent_ini(tmp_path: Path) -> None:
    missing = tmp_path / "RoboDK" / "settings.ini"
    with pytest.raises(SystemExit) as exc:
        install_app.write_python_setting(
            "/usr/bin/python3",
            running=False,
            settings_files=[missing],
        )
    assert "Open RoboDK once" in str(exc.value)
    assert not missing.exists()


def test_write_python_setting_refuses_when_running(tmp_path: Path) -> None:
    ini = tmp_path / "settings.ini"
    ini.write_text("Path_PythonRun2=/old\n", encoding="utf-8")
    with pytest.raises(SystemExit) as exc:
        install_app.write_python_setting(
            "/usr/bin/python3",
            running=True,
            settings_files=[ini],
        )
    assert "running" in str(exc.value).lower()
    assert "Path_PythonRun2=/old" in ini.read_text(encoding="utf-8")


def test_enable_addin_missing_ini_does_not_create(tmp_path: Path) -> None:
    missing = tmp_path / "AddinManager.ini"
    result = install_app.enable_addin(tmp_path / "addin-root", ini_files=[missing])
    assert result == "missing"
    assert not missing.exists()


def test_enable_addin_appends_enabled_path(tmp_path: Path) -> None:
    ami = tmp_path / "AddinManager.ini"
    ami.write_text("[Enabled]\nsize=1\n0=/other/addin\n\n[Watchdog]\nsize=0\n", encoding="utf-8")
    addin_root = tmp_path / "com.multiplylabs.app.ctnav"
    addin_root.mkdir()
    result = install_app.enable_addin(addin_root, ini_files=[ami])
    assert result == "enabled"
    text = ami.read_text(encoding="utf-8")
    assert "size=2" in text
    assert str(addin_root.resolve()).replace("\\", "/") in text


def test_enable_addin_already_listed(tmp_path: Path) -> None:
    addin_root = tmp_path / "com.multiplylabs.app.ctnav"
    addin_root.mkdir()
    listed = str(addin_root.resolve()).replace("\\", "/")
    ami = tmp_path / "AddinManager.ini"
    ami.write_text(f"[Enabled]\nsize=1\n0={listed}\n", encoding="utf-8")
    result = install_app.enable_addin(addin_root, ini_files=[ami])
    assert result == "already"


def test_venv_python_posix_and_windows(tmp_path: Path) -> None:
    assert install_app.venv_python(tmp_path, system="Linux") == tmp_path / "bin" / "python"
    assert install_app.venv_python(tmp_path, system="Windows") == tmp_path / "Scripts" / "python.exe"


def test_ensure_venv_creates_and_reuses(tmp_path: Path) -> None:
    venv_dir = tmp_path / "venv"
    first = install_app.ensure_venv(sys.executable, venv_dir)
    assert Path(first).is_file()
    marker = venv_dir / "created-by-test"
    marker.write_text("keep", encoding="utf-8")
    second = install_app.ensure_venv(sys.executable, venv_dir)
    assert second == first
    assert marker.is_file()


def test_ensure_venv_missing_creator(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as exc:
        install_app.ensure_venv(str(tmp_path / "no-such-python"), tmp_path / "venv")
    message = str(exc.value)
    assert "python3-venv" in message
    assert "Apple /usr/bin/python3" in message


def test_pyside_linux_hint_names_apt_packages() -> None:
    hint = install_app.pyside_load_failure_hint(system="Linux")
    assert "sudo apt install" in hint
    assert "libxcb-cursor0" in hint
    assert "sudo dnf install" in hint
