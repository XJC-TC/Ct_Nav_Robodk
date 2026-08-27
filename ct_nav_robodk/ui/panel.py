"""The CtNav panel: pick a ct_config navigation node, watch the station do it.

Run standalone with RoboDK already open:

    python -m ct_nav_robodk.ui.panel

Playback is pumped by a ``QTimer`` on the GUI thread rather than a worker thread. It
looks like the harder choice but is the simpler one: every RoboDK call stays on one
thread, Stop needs no synchronization, and the panel stays responsive because each
timer tick applies a single interpolation frame and returns.
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path

from PySide6 import QtCore, QtWidgets

from ct_nav import (
    ArmConfig,
    ClusterConfig,
    ConfigError,
    Mode,
    Plan,
    PlanError,
    StepKind,
    Visit,
    discover_cluster_dir,
    load_cluster,
    plan_move,
    plan_park,
)
from ct_nav.park_poses import PARK_POSES

from ..connection import ConnectionError_, active_station_name, connect
from ..driver import (
    Driver,
    DriverError,
    DriverOptions,
    nearest_highway_node,
    read_rail_pose,
    read_rail_robodk,
)
from ..eoat import EoatError, apply_eoat, list_eoats
from ..path_trace import PathMonitor
from ..program_export import ExportError, export_plan
from ..station_map import (
    LOCAL_MAP_NAME,
    StationMapError,
    calibrate_offset,
    load_default_station_map,
    save_station_map,
    with_rail_offset,
)

SETTINGS_ORG = "MultiplyLabs"
SETTINGS_APP = "CtNav"

# One frame per tick at roughly 60 Hz; the driver decides how many frames a step needs.
TICK_MS = 16


class CtNavPanel(QtWidgets.QWidget):
    def __init__(self, rdk=None, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("CtNav - ct_config navigation")
        # Float above RoboDK so clicking the 3D view does not bury the panel.
        # WindowStaysOnTopHint is system-level, so it works across RoboDK's process.
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

        self._rdk = rdk
        self._driver: Driver | None = None
        self._path_monitor: PathMonitor | None = None
        self._cluster: ClusterConfig | None = None
        self._plan: Plan | None = None
        self._running_plan: Plan | None = None
        self._frames = None
        self._current_step = -1
        self._motion_highlight: int | None = None
        self._motion_finish_highway: str | None = None
        self._pending_step_index = -1
        self._swallow_step_click = False
        self._settings = QtCore.QSettings(SETTINGS_ORG, SETTINGS_APP)

        # Remembered per arm so switching arms and back does not lose where each one is.
        self._highway_state: dict[str, str] = {}

        self._build_ui()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._on_tick)
        self._step_click_timer = QtCore.QTimer(self)
        self._step_click_timer.setSingleShot(True)
        self._step_click_timer.timeout.connect(self._on_step_click_timeout)

        discovered = discover_cluster_dir()
        self.cluster_edit.setText(
            self._settings.value(
                "cluster_dir", str(discovered) if discovered else "", type=str
            )
        )
        QtCore.QTimer.singleShot(0, self._initial_load)

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        layout.addWidget(self._build_cluster_box())
        layout.addWidget(self._build_selection_box())
        layout.addWidget(self._build_position_box())
        layout.addWidget(self._build_eoat_box())
        layout.addWidget(self._build_steps_box(), stretch=1)
        layout.addLayout(self._build_action_row())

        self.status = QtWidgets.QLabel("Starting up ...")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

    def _build_cluster_box(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("ct_config cluster")
        row = QtWidgets.QHBoxLayout(box)

        self.cluster_edit = QtWidgets.QLineEdit()
        self.cluster_edit.setPlaceholderText("e.g. ~/Bitbucket/ct_config/azula1")
        browse = QtWidgets.QPushButton("Browse ...")
        browse.clicked.connect(self._on_browse)
        reload_button = QtWidgets.QPushButton("Reload")
        reload_button.clicked.connect(self._on_reload)

        row.addWidget(self.cluster_edit, stretch=1)
        row.addWidget(browse)
        row.addWidget(reload_button)
        return box

    def _build_selection_box(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("Target")
        box.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum
        )
        grid = QtWidgets.QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(4)

        self.arm_combo = QtWidgets.QComboBox()
        self.module_combo = QtWidgets.QComboBox()
        self.target_combo = QtWidgets.QComboBox()
        self.node_combo = QtWidgets.QComboBox()
        self.visit_combo = QtWidgets.QComboBox()
        for visit in (Visit.ENTER, Visit.PICK_PLACE, Visit.EXIT):
            self.visit_combo.addItem(visit.label, visit.value)
        self.mode_combo = QtWidgets.QComboBox()
        for mode in Mode:
            # Store the plain value: Qt round-trips user data through QVariant, which
            # turns a str-backed enum member into a bare str anyway.
            self.mode_combo.addItem(mode.label, mode.value)

        pairs = (
            ("Arm", self.arm_combo, "Module", self.module_combo),
            ("Target", self.target_combo, "Node", self.node_combo),
            ("Visit", self.visit_combo, "Mode", self.mode_combo),
        )
        for row, (left_label, left, right_label, right) in enumerate(pairs):
            grid.addWidget(QtWidgets.QLabel(left_label), row, 0)
            grid.addWidget(left, row, 1)
            grid.addWidget(QtWidgets.QLabel(right_label), row, 2)
            grid.addWidget(right, row, 3)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

        self.arm_combo.currentIndexChanged.connect(self._on_arm_changed)
        self.module_combo.currentIndexChanged.connect(self._on_module_changed)
        self.target_combo.currentIndexChanged.connect(self._on_target_changed)
        self.node_combo.currentIndexChanged.connect(self._refresh_plan)
        self.visit_combo.currentIndexChanged.connect(self._refresh_plan)
        self.mode_combo.currentIndexChanged.connect(self._refresh_plan)
        return box

    def _build_position_box(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("Current position")
        grid = QtWidgets.QGridLayout(box)

        self.highway_combo = QtWidgets.QComboBox()
        self.highway_combo.setToolTip(
            "Which highway node the arm is standing on. Full navigation plans the rail "
            "route from here."
        )
        self.highway_combo.currentIndexChanged.connect(self._on_highway_changed)

        sync = QtWidgets.QPushButton("Sync from station")
        sync.setToolTip("Pick the highway node closest to where the rails actually are")
        sync.clicked.connect(self._on_sync)

        self.rail_label = QtWidgets.QLabel("-")

        grid.addWidget(QtWidgets.QLabel("Highway node"), 0, 0)
        grid.addWidget(self.highway_combo, 0, 1)
        grid.addWidget(sync, 0, 2)
        grid.addWidget(QtWidgets.QLabel("Rails (ct_config mm)"), 1, 0)
        grid.addWidget(self.rail_label, 1, 1, 1, 2)
        grid.setColumnStretch(1, 1)
        return box

    def _build_eoat_box(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("EOAT")
        grid = QtWidgets.QGridLayout(box)

        self.eoat_combo = QtWidgets.QComboBox()
        self.eoat_combo.setToolTip(
            "Tools parented under this MHR. Applying one hides every other EOAT and "
            "leaves CABLE GUARD visible."
        )
        apply_button = QtWidgets.QPushButton("Apply")
        apply_button.setToolTip("Show the selected EOAT; hide the rest; keep CABLE GUARD")
        apply_button.clicked.connect(self._on_apply_eoat)
        self._eoat_apply_button = apply_button
        refresh_button = QtWidgets.QPushButton("Refresh")
        refresh_button.clicked.connect(self._refresh_eoats)
        self._eoat_refresh_button = refresh_button
        self.eoat_guard_label = QtWidgets.QLabel("-")
        self.eoat_guard_label.setWordWrap(True)

        grid.addWidget(QtWidgets.QLabel("Fitted"), 0, 0)
        grid.addWidget(self.eoat_combo, 0, 1)
        grid.addWidget(apply_button, 0, 2)
        grid.addWidget(refresh_button, 0, 3)
        grid.addWidget(QtWidgets.QLabel("Always on"), 1, 0)
        grid.addWidget(self.eoat_guard_label, 1, 1, 1, 3)
        grid.setColumnStretch(1, 1)
        return box

    def _build_steps_box(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("Planned steps")
        column = QtWidgets.QVBoxLayout(box)

        self.route_label = QtWidgets.QLabel("-")
        self.route_label.setWordWrap(True)
        self.steps_list = QtWidgets.QListWidget()
        self.steps_list.setAlternatingRowColors(True)
        self.steps_list.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.steps_list.itemClicked.connect(self._on_step_clicked)
        self.steps_list.itemDoubleClicked.connect(self._on_step_double_clicked)
        self.steps_list.setToolTip(
            "Click a step to joint-move to it; double-click to jump there instantly."
        )

        self.warnings_label = QtWidgets.QLabel()
        self.warnings_label.setWordWrap(True)
        self.warnings_label.setStyleSheet("color: #b06000;")
        self.warnings_label.hide()

        column.addWidget(self.route_label)
        column.addWidget(self.steps_list, stretch=1)
        column.addWidget(self.warnings_label)
        return box

    def _build_action_row(self) -> QtWidgets.QLayout:
        outer = QtWidgets.QVBoxLayout()

        speed_row = QtWidgets.QHBoxLayout()
        self.animate_check = QtWidgets.QCheckBox("Animate")
        self.animate_check.setChecked(True)
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 20.0)
        self.speed_spin.setSingleStep(0.5)
        self.speed_spin.setValue(0.5)
        self.speed_spin.setSuffix("x")
        speed_row.addWidget(self.animate_check)
        speed_row.addWidget(QtWidgets.QLabel("Speed"))
        speed_row.addWidget(self.speed_spin)
        speed_row.addStretch(1)
        outer.addLayout(speed_row)

        path_row = QtWidgets.QHBoxLayout()
        self.path_check_box = QtWidgets.QCheckBox("Path cubes")
        self.path_check_box.setToolTip(
            "Wrap the UR body and visible EOAT/cable guard in CAD-hugging cubes, "
            "and leave a coarse trail of swept space. Rails are not wrapped. "
            "Collision stopping is a separate checkbox."
        )
        self.path_check_box.setChecked(
            self._settings.value("path_cubes_enabled", False, type=bool)
        )
        self.path_check_box.toggled.connect(self._on_path_cubes_toggled)
        self.collision_check_box = QtWidgets.QCheckBox("Collision")
        self.collision_check_box.setToolTip(
            "Stop if a new cube overlaps another entity (cell CAD, another robot, "
            "or another arm's trail). The moving arm's own body at the start pose "
            "is ignored. Hits are red. Off by default."
        )
        self.collision_check_box.setChecked(False)
        self.collision_check_box.setEnabled(self.path_check_box.isChecked())
        self.collision_check_box.toggled.connect(self._on_collision_check_toggled)
        clear_paths = QtWidgets.QPushButton("Clear paths")
        clear_paths.setToolTip("Remove every CtNav colour-block trail from the station")
        clear_paths.clicked.connect(self._on_clear_paths)
        self._clear_paths_button = clear_paths
        path_row.addWidget(self.path_check_box)
        path_row.addWidget(self.collision_check_box)
        path_row.addWidget(clear_paths)
        path_row.addStretch(1)
        outer.addLayout(path_row)

        buttons = QtWidgets.QHBoxLayout()
        self.go_button = QtWidgets.QPushButton("Go")
        self.go_button.setDefault(True)
        self.go_button.clicked.connect(self._on_go)
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._on_stop)
        self.export_button = QtWidgets.QPushButton("Export as Program")
        self.export_button.clicked.connect(self._on_export)
        buttons.addWidget(self.go_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.export_button)
        outer.addLayout(buttons)

        extras = QtWidgets.QHBoxLayout()
        self.park_combo = QtWidgets.QComboBox()
        self.park_combo.addItems(sorted(PARK_POSES["lower"]))
        park_button = QtWidgets.QPushButton("Reset to PARK")
        park_button.clicked.connect(self._on_park)
        calibrate_button = QtWidgets.QPushButton("Calibrate rail ...")
        calibrate_button.clicked.connect(self._on_calibrate)
        extras.addWidget(self.park_combo, stretch=1)
        extras.addWidget(park_button)
        extras.addWidget(calibrate_button)
        outer.addLayout(extras)
        return outer

    # ------------------------------------------------------------------
    # Connection and config
    # ------------------------------------------------------------------

    def _initial_load(self) -> None:
        self._connect_station()
        self._on_reload()

    def _connect_station(self) -> bool:
        if self._driver is not None:
            return True
        try:
            if self._rdk is None:
                self._rdk = connect()
            station_map = load_default_station_map()
        except (ConnectionError_, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return False

        self._driver = Driver(self._rdk, station_map, DriverOptions())
        self._path_monitor = PathMonitor(self._rdk)
        problems = self._driver.verify()
        station = active_station_name(self._rdk) or "(no station)"
        if problems:
            self._set_status(
                f"Station {station}: " + " | ".join(problems), error=True
            )
        else:
            self._set_status(f"Connected to {station}, {station_map.station} map loaded.")
        return True

    def _on_browse(self) -> None:
        chosen = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select a ct_config cluster folder", self.cluster_edit.text()
        )
        if chosen:
            self.cluster_edit.setText(chosen)
            self._on_reload()

    def _on_reload(self) -> None:
        path = Path(self.cluster_edit.text().strip())
        try:
            self._cluster = load_cluster(path)
        except ConfigError as exc:
            self._cluster = None
            self._set_status(str(exc), error=True)
            self._repopulate_arms()
            return

        self._settings.setValue("cluster_dir", str(path))
        self._repopulate_arms()
        self._set_status(
            f"Loaded {self._cluster.name}: {', '.join(self._cluster.arm_names())}"
        )

    # ------------------------------------------------------------------
    # Cascading selection
    # ------------------------------------------------------------------

    def _repopulate_arms(self) -> None:
        with _blocked(self.arm_combo):
            self.arm_combo.clear()
            if self._cluster is not None:
                self.arm_combo.addItems(self._cluster.arm_names())
        self._on_arm_changed()

    def current_arm(self) -> ArmConfig | None:
        if self._cluster is None or not self.arm_combo.count():
            return None
        try:
            return self._cluster.arm(self.arm_combo.currentText())
        except ConfigError:
            return None

    def _on_arm_changed(self) -> None:
        arm = self.current_arm()
        with _blocked(self.module_combo):
            self.module_combo.clear()
            if arm is not None:
                self.module_combo.addItems(arm.modules())
        self._repopulate_highway(arm)
        self._refresh_eoats()
        self._on_module_changed()

    def _repopulate_highway(self, arm: ArmConfig | None) -> None:
        with _blocked(self.highway_combo):
            self.highway_combo.clear()
            if arm is None:
                return
            self.highway_combo.addItems(sorted(arm.highway))
            remembered = self._highway_state.get(arm.name) or arm.highway_root()
            index = self.highway_combo.findText(remembered)
            if index >= 0:
                self.highway_combo.setCurrentIndex(index)
        self._refresh_rail_label()

    def _on_module_changed(self) -> None:
        arm = self.current_arm()
        with _blocked(self.target_combo):
            self.target_combo.clear()
            if arm is not None and self.module_combo.currentText():
                self.target_combo.addItems(arm.targets(self.module_combo.currentText()))
        self._on_target_changed()

    def _on_target_changed(self) -> None:
        arm = self.current_arm()
        with _blocked(self.node_combo):
            self.node_combo.clear()
            module = self.module_combo.currentText()
            target = self.target_combo.currentText()
            if arm is not None and module and target:
                try:
                    tree = arm.tree(arm.nav_target(module, target))
                except ConfigError as exc:
                    self._set_status(str(exc), error=True)
                else:
                    self.node_combo.addItems(list(tree.nodes))
                    # Approach nodes come first in the YAML, so land on the real
                    # destination (pick_place_node, tucked_away, or the last leaf).
                    preferred = tree.preferred_node()
                    if preferred:
                        index = self.node_combo.findText(preferred)
                        if index >= 0:
                            self.node_combo.setCurrentIndex(index)
        self._refresh_plan()

    def _on_highway_changed(self) -> None:
        arm = self.current_arm()
        if arm is not None and self.highway_combo.currentText():
            self._highway_state[arm.name] = self.highway_combo.currentText()
        self._refresh_plan()

    def _refresh_eoats(self) -> None:
        with _blocked(self.eoat_combo):
            self.eoat_combo.clear()
            self.eoat_combo.addItem("(none — cable guard only)", "")
        self.eoat_guard_label.setText("-")

        arm = self.current_arm()
        if arm is None or not self._connect_station():
            return
        try:
            items = self._driver.items_for(arm.name)
            inventory = list_eoats(self._rdk, items.robot, arm.name)
        except (DriverError, StationMapError, Exception) as exc:
            self.eoat_guard_label.setText(f"unavailable ({exc})")
            return

        guards = ", ".join(item.name for item in inventory.keep_visible) or "no CABLE GUARD found"
        self.eoat_guard_label.setText(guards)
        with _blocked(self.eoat_combo):
            for item in inventory.swappable:
                self.eoat_combo.addItem(item.name, item.name)
            active = inventory.active or ""
            index = self.eoat_combo.findData(active)
            if index >= 0:
                self.eoat_combo.setCurrentIndex(index)

    def _on_apply_eoat(self) -> None:
        arm = self.current_arm()
        if arm is None or not self._connect_station():
            return
        name = self.eoat_combo.currentData()
        chosen = name if name else None
        try:
            items = self._driver.items_for(arm.name)
            inventory = apply_eoat(self._rdk, items.robot, chosen)
        except (EoatError, DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return
        self._refresh_eoats()
        hidden = max(len(inventory.swappable) - (1 if chosen else 0), 0)
        if chosen:
            self._set_status(
                f"{arm.name}: fitted {chosen!r}; hid {hidden} other EOAT(s); "
                "CABLE GUARD left visible."
            )
        else:
            self._set_status(f"{arm.name}: all EOATs hidden except CABLE GUARD.")

    # ------------------------------------------------------------------
    # Planning and display
    # ------------------------------------------------------------------

    def _refresh_plan(self) -> None:
        self._step_click_timer.stop()
        self._plan = None
        self.steps_list.clear()
        self.route_label.setText("-")
        self.warnings_label.hide()

        arm = self.current_arm()
        module = self.module_combo.currentText()
        target = self.target_combo.currentText()
        node = self.node_combo.currentText()
        if arm is None or not (module and target and node):
            return

        try:
            plan = plan_move(
                arm,
                module,
                target,
                node,
                mode=Mode(self.mode_combo.currentData()),
                visit=Visit(self.visit_combo.currentData()),
                current_highway_node=self.highway_combo.currentText() or None,
            )
        except (ConfigError, PlanError) as exc:
            self._set_status(str(exc), error=True)
            return

        self._plan = plan
        self._show_plan(plan)

    def _show_plan(self, plan: Plan) -> None:
        self.route_label.setText(
            "highway: " + " -> ".join(plan.highway_route) if plan.highway_route else "no rails"
        )
        for index, step in enumerate(plan.steps):
            self.steps_list.addItem(f"{index + 1:>2}. {step.describe()}")
        if plan.warnings:
            self.warnings_label.setText("\n".join(plan.warnings))
            self.warnings_label.show()

    def _refresh_rail_label(self) -> None:
        arm = self.current_arm()
        if arm is None or self._driver is None:
            self.rail_label.setText("-")
            return
        try:
            pose = read_rail_pose(self._driver.items_for(arm.name))
        except (DriverError, StationMapError) as exc:
            self.rail_label.setText(f"unavailable ({exc})")
            return
        self.rail_label.setText(pose.describe() if not pose.is_empty() else "no rails")

    def _on_sync(self) -> None:
        arm = self.current_arm()
        if arm is None or not self._connect_station():
            return
        try:
            items = self._driver.items_for(arm.name)
        except (DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return

        node = nearest_highway_node(arm, items)
        self._refresh_rail_label()
        if node is None:
            self._set_status(f"{arm.name} has no rails, so there is no node to sync.")
            return
        index = self.highway_combo.findText(node)
        if index >= 0:
            self.highway_combo.setCurrentIndex(index)
        self._set_status(f"{arm.name} rails are closest to highway node {node!r}.")

    # ------------------------------------------------------------------
    # Playback
    # ------------------------------------------------------------------

    def _start(self, plan: Plan) -> None:
        if not self._connect_station():
            return
        try:
            self._driver.items_for(plan.arm)
        except (DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return

        self._attach_path_check(plan.arm, replace=True)
        if self.path_check_box.isChecked() and (
            self._path_monitor is None or self._path_monitor.arm != plan.arm
        ):
            return

        self._driver.options = DriverOptions(
            animate=self.animate_check.isChecked(), speed=self.speed_spin.value()
        )
        self._frames = self._driver.iter_frames(plan)
        self._running_plan = plan
        self._current_step = -1
        self._motion_highlight = None
        self._motion_finish_highway = plan.end_highway_node
        self._set_running(True)
        checking = ""
        if self.path_check_box.isChecked():
            checking = (
                " (path cubes + collision)"
                if self.collision_check_box.isChecked()
                else " (path cubes)"
            )
        self._set_status(f"Running {plan.label} ({len(plan.steps)} steps){checking} ...")
        self._timer.start()

    def _on_tick(self) -> None:
        if self._frames is None:
            self._finish("Idle.")
            return
        try:
            index, frame = next(self._frames)
        except StopIteration:
            plan = self._running_plan
            if self._motion_finish_highway:
                self._highway_state[plan.arm] = self._motion_finish_highway
                node_index = self.highway_combo.findText(self._motion_finish_highway)
                if node_index >= 0:
                    with _blocked(self.highway_combo):
                        self.highway_combo.setCurrentIndex(node_index)
            if self._motion_highlight is None:
                self._finish(f"Done: {plan.label}")
            else:
                step = plan.steps[self._motion_highlight]
                self._finish(f"Moved to step {self._motion_highlight + 1}: {step.label}")
            return
        except (DriverError, StationMapError) as exc:
            self._finish(str(exc), error=True)
            return

        try:
            self._driver.apply_step(self._running_plan.arm, frame)
        except (DriverError, StationMapError) as exc:
            self._finish(str(exc), error=True)
            return

        if index != self._current_step:
            self._current_step = index
            row = self._motion_highlight if self._motion_highlight is not None else index
            self.steps_list.setCurrentRow(row)

        if self.path_check_box.isChecked() and not self._poll_collision_or_stop():
            return

    def _on_go(self) -> None:
        if self._plan is None:
            self._set_status("Nothing selected to run.", error=True)
            return
        self._start(self._plan)

    def _on_stop(self) -> None:
        self._finish("Stopped part-way through; the arm is left where it halted.")

    def _finish(self, message: str, error: bool = False) -> None:
        self._timer.stop()
        self._step_click_timer.stop()
        self._frames = None
        self._motion_highlight = None
        self._motion_finish_highway = None
        if self._path_monitor is not None:
            self._path_monitor.stop()
            extra = []
            if self.path_check_box.isChecked():
                if self._path_monitor.painted:
                    extra.append(f"{self._path_monitor.painted} path cubes")
                if self._path_monitor.last_error:
                    extra.append(self._path_monitor.last_error)
            if extra:
                message = f"{message} ({'; '.join(extra)})"
        self._set_running(False)
        self._refresh_rail_label()
        self._set_status(message, error=error)

    def _set_running(self, running: bool) -> None:
        self.stop_button.setEnabled(running)
        for widget in (
            self.go_button,
            self.export_button,
            self.arm_combo,
            self.module_combo,
            self.target_combo,
            self.node_combo,
            self.visit_combo,
            self.mode_combo,
            self.highway_combo,
            self.eoat_combo,
            self._eoat_apply_button,
            self._eoat_refresh_button,
            self.path_check_box,
            self.collision_check_box,
            self._clear_paths_button,
        ):
            widget.setEnabled(not running)
        if not running:
            self.collision_check_box.setEnabled(self.path_check_box.isChecked())

    def _on_step_clicked(self, item: QtWidgets.QListWidgetItem) -> None:
        """Defer a joint-move until we know this is not the start of a double-click."""
        if self._swallow_step_click or self._plan is None or self._frames is not None:
            return
        self._pending_step_index = self.steps_list.row(item)
        self._step_click_timer.start(QtWidgets.QApplication.doubleClickInterval())

    def _on_step_click_timeout(self) -> None:
        self._animate_to_step(self._pending_step_index)

    def _on_step_double_clicked(self, item: QtWidgets.QListWidgetItem) -> None:
        """Jump straight to one step, for inspecting a pose without interpolating."""
        self._step_click_timer.stop()
        self._swallow_step_click = True
        QtCore.QTimer.singleShot(
            QtWidgets.QApplication.doubleClickInterval(),
            self._clear_swallow_step_click,
        )
        if self._plan is None or not self._connect_station() or self._frames is not None:
            return
        index = self.steps_list.row(item)
        try:
            self._driver.apply_step(self._plan.arm, self._plan.steps[index])
            self._attach_path_check(self._plan.arm, replace=False)
            hit = self._collision_message(index)
            if self._path_monitor is not None:
                self._path_monitor.stop()
        except (DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return
        self._sync_highway_from_step(self._plan.arm, self._plan.steps[index])
        self._refresh_rail_label()
        if hit:
            self._set_status(hit, error=True)
            return
        self._set_status(f"Jumped to step {index + 1}: {self._plan.steps[index].label}")

    def _clear_swallow_step_click(self) -> None:
        self._swallow_step_click = False

    def _animate_to_step(self, index: int) -> None:
        """Joint-interpolate from the station's current pose to one planned step."""
        if self._plan is None or not (0 <= index < len(self._plan.steps)):
            return
        if self._frames is not None or not self._connect_station():
            return
        step = self._plan.steps[index]
        try:
            self._driver.items_for(self._plan.arm)
        except (DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return

        self._driver.options = DriverOptions(animate=True, speed=self.speed_spin.value())
        self._frames = self._driver.iter_step_frames(self._plan.arm, [step])
        self._running_plan = self._plan
        self._current_step = -1
        self._motion_highlight = index
        self._motion_finish_highway = _highway_node_of(step)
        self._attach_path_check(self._plan.arm, replace=False)
        self._set_running(True)
        self._set_status(f"Joint-moving to step {index + 1}: {step.label} ...")
        self._timer.start()

    def _sync_highway_from_step(self, arm: str, step) -> None:
        node = _highway_node_of(step)
        if not node:
            return
        self._highway_state[arm] = node
        index = self.highway_combo.findText(node)
        if index >= 0:
            with _blocked(self.highway_combo):
                self.highway_combo.setCurrentIndex(index)

    def _on_park(self) -> None:
        arm = self.current_arm()
        if arm is None:
            return
        try:
            plan = plan_park(
                arm,
                self.park_combo.currentText(),
                current_highway_node=self.highway_combo.currentText() or None,
            )
        except PlanError as exc:
            self._set_status(str(exc), error=True)
            return
        self._start(plan)

    # ------------------------------------------------------------------
    # Export and calibration
    # ------------------------------------------------------------------

    def _on_export(self) -> None:
        if self._plan is None:
            self._set_status("Nothing selected to export.", error=True)
            return
        if not self._connect_station():
            return
        try:
            result = export_plan(self._rdk, self._driver.station_map, self._plan)
        except (ExportError, DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return

        message = f"Exported {result.summary()}"
        if result.notes:
            message += "\n" + "\n".join(result.notes)
        self._set_status(message)

    def _on_calibrate(self) -> None:
        arm = self.current_arm()
        if arm is None or not self._connect_station():
            return
        try:
            arm_map = self._driver.station_map.arm(arm.name)
        except StationMapError as exc:
            self._set_status(str(exc), error=True)
            return
        if not arm_map.rails:
            self._set_status(f"{arm.name} has no rails to calibrate.")
            return

        axis, ok = QtWidgets.QInputDialog.getItem(
            self, "Calibrate rail", "Rail axis", sorted(arm_map.rails), 0, False
        )
        if not ok:
            return

        try:
            items = self._driver.items_for(arm.name)
            observed = read_rail_robodk(items, axis)
        except (DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)
            return

        value, ok = QtWidgets.QInputDialog.getDouble(
            self,
            "Calibrate rail",
            f"RoboDK reads {observed:.3f} for the {axis} rail.\n"
            "What is this position in ct_config millimetres?",
            observed,
            -100000.0,
            100000.0,
            3,
        )
        if not ok:
            return

        rail = arm_map.rails[axis]
        offset = calibrate_offset(rail, observed, value)
        updated = with_rail_offset(self._driver.station_map, arm.name, axis, offset)
        target = Path(__file__).resolve().parent.parent.parent / LOCAL_MAP_NAME
        try:
            save_station_map(updated, target)
        except OSError as exc:
            self._set_status(f"Could not write {target}: {exc}", error=True)
            return

        self._driver.station_map = updated
        self._driver.forget()
        self._refresh_rail_label()
        self._refresh_plan()
        self._set_status(
            f"{arm.name} {axis} rail offset set to {offset:.3f} and saved to {target.name}."
        )

    # ------------------------------------------------------------------

    def _on_path_cubes_toggled(self, checked: bool) -> None:
        self._settings.setValue("path_cubes_enabled", checked)
        if not checked:
            self.collision_check_box.setChecked(False)
        self.collision_check_box.setEnabled(checked and self.go_button.isEnabled())

    def _on_collision_check_toggled(self, checked: bool) -> None:
        self._settings.setValue("path_stop_on_collision", checked)

    def _on_clear_paths(self) -> None:
        if self._path_monitor is None and not self._connect_station():
            return
        if self._path_monitor is None:
            self._path_monitor = PathMonitor(self._rdk)
        self._path_monitor.clear()
        self._set_status("Cleared CtNav path traces.")

    def _attach_path_check(self, arm: str, *, replace: bool) -> None:
        if not self.path_check_box.isChecked() or self._driver is None:
            return
        if self._path_monitor is None:
            if not self._connect_station():
                return
            self._path_monitor = PathMonitor(self._rdk)
        try:
            items = self._driver.items_for(arm)
            self._path_monitor.attach(arm, items, replace=replace)
        except (DriverError, StationMapError) as exc:
            self._set_status(str(exc), error=True)

    def _poll_collision_or_stop(self) -> bool:
        """True to keep playing. False if a cube already overlapped an entity."""
        message = self._collision_message()
        if message:
            self._finish(message, error=True)
            return False
        return True

    def _collision_message(self, step_index: int | None = None) -> str | None:
        """A status line when the current pose collides; None if checking is off or clear."""
        if (
            not self.path_check_box.isChecked()
            or self._path_monitor is None
            or self._path_monitor.arm is None
        ):
            return None
        report = self._path_monitor.observe(
            check_collision=self.collision_check_box.isChecked()
        )
        if not report.hit:
            return None
        index = self._current_step if step_index is None else step_index
        plan = self._running_plan or self._plan
        if plan is not None and 0 <= index < len(plan.steps):
            step = plan.steps[index]
            return f"Collision at step {index + 1} [{step.label}]: {report.describe()}"
        return f"Collision: {report.describe()}"

    def _set_status(self, message: str, error: bool = False) -> None:
        self.status.setStyleSheet("color: #c00000;" if error else "")
        self.status.setText(message)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt naming
        self._step_click_timer.stop()
        self._timer.stop()
        if self._path_monitor is not None:
            self._path_monitor.stop()
            self._path_monitor.clear()
        super().closeEvent(event)


def _highway_node_of(step) -> str | None:
    """Highway node named in a highway step's label, if this is one."""
    if step.kind is StepKind.HIGHWAY and step.label.startswith("highway "):
        return step.label[len("highway ") :]
    return None


class _blocked:
    """Suppress a widget's change signals while it is being repopulated."""

    def __init__(self, widget: QtWidgets.QWidget) -> None:
        self._widget = widget

    def __enter__(self) -> QtWidgets.QWidget:
        self._previous = self._widget.blockSignals(True)
        return self._widget

    def __exit__(self, *exc_info) -> None:
        self._widget.blockSignals(self._previous)


def main() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    try:
        panel = CtNavPanel()
    except Exception:
        traceback.print_exc()
        return 1
    panel.resize(620, 820)
    panel.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
