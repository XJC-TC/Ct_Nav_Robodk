# CtNav (RoboDK App)

Toolbar button that opens a panel for driving the station's arms through the navigation
nodes defined in a [ct_config](https://bitbucket.org/) cluster.

This folder is the App *shell*. The `ct_nav` / `ct_nav_robodk` packages, `station_map.yaml`
and `requirements.txt` are vendored in next to `CtNavPanel.py` by
`scripts/build_package.py`, so the installed copy is self-contained. Do not edit the
installed copy -- edit the repo and re-run `python3 scripts/install_app.py --sync`.

## Install

Quit RoboDK, then from the repository root:

```bash
python3 scripts/install_app.py -y
```

On Windows, `python scripts/install_app.py -y` is the same entry. Linux and macOS install
only into the user-level Add-ins folder; Windows still also copies into install-root
`Apps/`.

If this account has never opened RoboDK, open it once, quit, and re-run so `settings.ini`
and `AddinManager.ini` exist for the installer to patch.

Linux / macOS toolbar click has not been verified on a real RoboDK in this pass.

## If the button does nothing

Almost always one of two things:

- The Python interpreter in RoboDK's options is a bare name (`python`) rather than an
  absolute path. RoboDK spawns Action scripts with a minimal `PATH`, so nothing runs and
  no error is shown.
- The interpreter cannot load PySide6 (Anaconda Qt clash on Windows; missing `libxcb-*`
  on Linux). Use the interpreter the installer printed, or the `{repo}/.venv` it created.

See the repository `README.md` for the full setup and rail calibration notes.
