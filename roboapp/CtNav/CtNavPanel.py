# --------------------------------------------
# --------------- DESCRIPTION ----------------
#
# CtNav: drive a station's arms through the navigation nodes defined in ct_config.
#
# Checkable action: click the toolbar/menu button to open the panel, click again to
# close it.
#
# Two per-machine RoboDK traps apply here, both inherited from the sibling
# Robodk_Auto_Attach project where they were diagnosed:
#
# 1. RoboDK's `Tools > Options > Python > Python interpreter` must hold an ABSOLUTE
#    path. RoboDK spawns Action scripts with a minimal PATH, so a bare "python" /
#    "python3" resolves to nothing and the script silently never starts -- the button
#    appears to do nothing at all. Type the full path in directly; the "Select..."
#    picker only offers bare command names.
# 2. Anaconda base usually has conda Qt5 (`qt-main` / `pyqt`) ahead of a pip-installed
#    PySide6, which fails with "DLL load failed while importing QtCore". Point RoboDK
#    at a standalone CPython instead. `scripts/install_app.py` probes for one.
#
# More information on RoboDK Apps here:
#     https://github.com/RoboDK/Plug-In-Interface/tree/master/PluginAppLoader
#
# --------------------------------------------

import os
import sys

# RoboDK sets its own PYTHONPATH (pointing at its bundled Python, which ships an older
# `robodk` copy) before spawning Action scripts. A plain `import robodk` would then
# shadow the pip-installed one this project is written against. Demote -- not remove,
# so it stays a fallback -- anything that arrived via PYTHONPATH to the back of
# sys.path, before the first `import robodk` anywhere in the process. PYTHONPATH's
# entries land on sys.path with the trailing slash stripped, so both sides have to be
# normalized or the comparison silently matches nothing.
_pythonpath_dirs = {
    os.path.normpath(p) for p in os.environ.get("PYTHONPATH", "").split(os.pathsep) if p
}
if _pythonpath_dirs:
    sys.path = [p for p in sys.path if os.path.normpath(p) not in _pythonpath_dirs] + [
        p for p in sys.path if os.path.normpath(p) in _pythonpath_dirs
    ]

from robodk import robolink, roboapps  # noqa: E402

ACTION_NAME = os.path.basename(__file__)


def _prepare_pyside6_native_libs():
    """Prefer PySide6's own Qt6 DLLs over conda/Anaconda Qt5 on PATH (Windows).

    Fixes some machines outright; a clean non-conda interpreter is still the reliable
    fix when conda Qt is badly entangled.
    """
    try:
        import PySide6
    except ImportError:
        return
    root = os.path.dirname(PySide6.__file__)
    if hasattr(os, "add_dll_directory"):
        try:
            os.add_dll_directory(root)
            shiboken = os.path.join(os.path.dirname(root), "shiboken6")
            if os.path.isdir(shiboken):
                os.add_dll_directory(shiboken)
        except OSError:
            pass
    os.environ["PATH"] = root + os.pathsep + os.environ.get("PATH", "")
    for platforms in (
        os.path.join(root, "plugins", "platforms"),
        os.path.join(root, "Qt", "plugins", "platforms"),
    ):
        if os.path.isdir(platforms):
            os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", platforms)
            break


def ActionChecked():
    """Open the panel and block until the user unchecks the toolbar button."""
    _prepare_pyside6_native_libs()
    from PySide6 import QtCore, QtWidgets

    from ct_nav_robodk.ui.panel import CtNavPanel

    RDK = robolink.Robolink()
    APP = roboapps.RunApplication()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = CtNavPanel(rdk=RDK)
    panel.resize(560, 780)
    panel.show()

    # Bridge roboapps' "is the toolbar button still checked?" state with Qt's event
    # loop: poll it on a timer and quit the Qt loop when the user unchecks the button.
    watchdog = QtCore.QTimer()
    watchdog.timeout.connect(lambda: None if APP.Run() else app.quit())
    watchdog.start(200)

    app.exec()


def ActionUnchecked():
    """No extra cleanup needed: unchecking stops APP.Run() above, which quits the Qt
    loop and lets ActionChecked() return -- the window is destroyed with it."""
    pass


def runmain():
    if roboapps.Unchecked():
        ActionUnchecked()
    else:
        roboapps.SkipKill()  # the panel can stay open indefinitely; don't force-kill it
        ActionChecked()


if __name__ == "__main__":
    runmain()
