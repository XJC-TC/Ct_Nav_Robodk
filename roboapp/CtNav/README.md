# CtNav (RoboDK App)

Toolbar button that opens a panel for driving the station's arms through the navigation
nodes defined in a [ct_config](https://bitbucket.org/) cluster.

This folder is the App *shell*. The `ct_nav` / `ct_nav_robodk` packages, `station_map.yaml`
and `requirements.txt` are vendored in next to `CtNavPanel.py` by
`scripts/build_package.py`, so the installed copy is self-contained. Do not edit the
installed copy -- edit the repo and re-run `python scripts/install_app.py --sync`.

## Install

From the repository root:

```bash
python scripts/install_app.py
```

Then set `Tools > Options > Python > Python interpreter` in RoboDK to the **absolute**
path the installer prints, and restart RoboDK.

## If the button does nothing

Almost always one of two things:

- The Python interpreter in RoboDK's options is a bare name (`python`) rather than an
  absolute path. RoboDK spawns Action scripts with a minimal `PATH`, so nothing runs and
  no error is shown.
- The interpreter is Anaconda base, where conda's Qt5 shadows pip's PySide6 and
  `from PySide6 import QtCore` fails with a DLL error. Use a standalone CPython.

See the repository `README.md` for the full setup and rail calibration notes.
