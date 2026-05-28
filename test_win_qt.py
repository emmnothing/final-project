from __future__ import annotations

import argparse
import json
import math
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Set, Tuple

try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    np = None
    HAVE_NUMPY = False

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    serial = None
    list_ports = None
    HAVE_SERIAL = False

try:
    import pyqtgraph as pg
    from PySide6 import QtCore, QtGui, QtWidgets
    HAVE_QT = True
except ImportError:
    pg = None
    QtCore = None
    QtGui = None
    QtWidgets = None
    HAVE_QT = False

if not HAVE_QT:
    class _DummyBase:
        pass

    class _DummyQt:
        SolidLine = 1
        DashLine = 2
        TextSelectableByMouse = 1

    class _DummyQtCore:
        Qt = _DummyQt()

    class _DummyQtGui:
        QCloseEvent = object

    class _DummyQtWidgets:
        QFrame = _DummyBase
        QMainWindow = _DummyBase
        QWidget = _DummyBase

    class _DummyPg:
        GraphicsLayoutWidget = _DummyBase

    QtCore = _DummyQtCore()
    QtGui = _DummyQtGui()
    QtWidgets = _DummyQtWidgets()
    pg = _DummyPg()


DEFAULT_PORT = "COM6"
DEFAULT_BAUDRATE = 961200
DEFAULT_WIDTH = 80
DEFAULT_HEIGHT = 80
DEFAULT_CELL_MM = 50
POLL_MAX_LINES = 1000
LOG_MAX_LINES = 500
SETTINGS_PATH = Path.home() / ".stm32_map_viewer.json"

MAZE_CELLS = 5
MAZE_CELL_MM = 700
MAZE_START_X_MM = -1500
MAZE_START_Y_MM = -1500
MAZE_START_CELL = (0, 0)
MAZE_EXIT_CELL = (4, 4)

COLOR_UNKNOWN = (24, 27, 33, 255)
COLOR_FREE = (238, 242, 246, 255)
COLOR_OCCUPIED = (222, 66, 74, 255)
COLOR_GRID = "#4a515d"
COLOR_MAZE_GRID = "#7c8795"
COLOR_ROBOT = "#25c26e"
COLOR_TARGET = "#ff9f1c"
COLOR_TARGET_DONE = "#36d399"
COLOR_TARGET_FAILED = "#ff4d6d"
COLOR_NAV_LINE = "#ffd166"
COLOR_TRACE = "#ffd166"

MAP_HEADER_RE = re.compile(
    r"^MAP\s+(?P<state>\w+)\s+w=(?P<w>\d+)\s+h=(?P<h>\d+)\s+"
    r"cell=(?P<cell>\d+)mm\s+rev=(?P<rev>\d+)"
)
MAP_ROW_RE = re.compile(r"^MAP\s+ROW\s+y=(?P<y>\d+)\s+rev=(?P<rev>\d+)\s+data=(?P<data>[?.#]+)")
MAP_STAT_RE = re.compile(r"^MAP\s+STAT\s+(?P<body>.*)$")
MAP_DIAG_RE = re.compile(r"^MAP\s+DIAG\s+(?P<body>.*)$")
MAP_PERF_RE = re.compile(r"^MAP\s+PERF\s+(?P<body>.*)$")
NAV_STAT_RE = re.compile(r"^NAV\s+STAT\s+(?P<body>.*)$")
POSE_RE = re.compile(r"\bpose=(-?\d+),(-?\d+),(-?\d+)")
POSE_LINE_RE = re.compile(r"^POSE\s+(-?\d+),(-?\d+),(-?\d+)(?:\s+state=([A-Z_]+))?$")
POSE_STATE_RE = re.compile(r"\bstate=([A-Z_]+)")
NAV_TARGET_RE = re.compile(r"\btarget=(-?\d+),(-?\d+)")
NAV_TARGET_NONE_RE = re.compile(r"\btarget=--")
NAV_ACTIVE_RE = re.compile(r"\bactive=(\d+)")
NAV_STATE_RE = re.compile(r"\bstate=([A-Z]+)")
NAV_INDEX_RE = re.compile(r"\bidx=(\d+)\s+len=(\d+)")
NAV_FRONT_RE = re.compile(r"\bfront=(\d+)")
NAV_BLOCK_RE = re.compile(r"\bblock=(\d+)")
NAV_RUN_RE = re.compile(r"^NAV\s+RUN\s+start=(\d+),(\d+)\s+goal=(\d+),(\d+)\s+len=(\d+)")
NAV_REPLAN_RE = re.compile(r"^NAV\s+REPLAN\s+\S+\s+from=(\d+),(\d+)\s+goal=(\d+),(\d+)\s+len=(\d+)")
NAV_LEG_RE = re.compile(r"^NAV\s+LEG\s+idx=(\d+)\s+cell=(\d+),(\d+)\s+target=(-?\d+),(-?\d+)\s+head=(-?\d+)")
NAV_BLOCKED_RE = re.compile(r"^NAV\s+BLOCKED\s+front=(\d+)\s+safe=(\d+)\s+angle=(\d+)\s+action=(\w+)")
NAV_START_CELL_RE = re.compile(r"^NAV\s+START\s+CELL\s+(\d+),(\d+)")
NAV_GOAL_CELL_RE = re.compile(r"^NAV\s+GOAL\s+CELL\s+(\d+),(\d+)")


def maze_cell_center(cell: Tuple[int, int]) -> Tuple[int, int]:
    origin_x = MAZE_START_X_MM - int((MAZE_START_CELL[0] + 0.5) * MAZE_CELL_MM)
    origin_y = MAZE_START_Y_MM - int((MAZE_START_CELL[1] + 0.5) * MAZE_CELL_MM)
    return (
        origin_x + int((cell[0] + 0.5) * MAZE_CELL_MM),
        origin_y + int((cell[1] + 0.5) * MAZE_CELL_MM),
    )


def clamp_maze_cell(value: object, fallback: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        number = fallback
    return max(0, min(MAZE_CELLS - 1, number))


def normalize_cell(value: object, fallback: Tuple[int, int]) -> Tuple[int, int]:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return (
            clamp_maze_cell(value[0], fallback[0]),
            clamp_maze_cell(value[1], fallback[1]),
        )
    return fallback


def load_viewer_settings() -> Tuple[Tuple[int, int], Tuple[int, int]]:
    try:
        data = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        data = {}
    return (
        normalize_cell(data.get("start_cell"), MAZE_START_CELL),
        normalize_cell(data.get("goal_cell"), MAZE_EXIT_CELL),
    )


def save_viewer_settings(start_cell: Tuple[int, int], goal_cell: Tuple[int, int]) -> None:
    data = {
        "start_cell": [int(start_cell[0]), int(start_cell[1])],
        "goal_cell": [int(goal_cell[0]), int(goal_cell[1])],
    }
    try:
        SETTINGS_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")
    except OSError:
        pass


def command_payload(cmd: str) -> bytes:
    cmd = cmd.strip()
    if not cmd:
        return b""
    if cmd in {"0", "1", "2", "3"}:
        return cmd.encode()
    if cmd == "91":
        return b"91\r\n"
    return (cmd + "\r\n").encode()


@dataclass
class MapModel:
    width: int = DEFAULT_WIDTH
    height: int = DEFAULT_HEIGHT
    cell_mm: int = DEFAULT_CELL_MM
    revision: int = 0
    state: str = "IDLE"
    rows: List[str] = field(default_factory=list)
    pending_seen: Set[int] = field(default_factory=set)
    nav_start_cell: Tuple[int, int] = MAZE_START_CELL
    nav_goal_cell: Tuple[int, int] = MAZE_EXIT_CELL
    nav_target: Optional[Tuple[int, int]] = None
    nav_active: bool = False
    nav_state: str = "IDLE"
    nav_index: int = 0
    nav_length: int = 0
    nav_front_mm: int = 0
    nav_blocked: bool = False
    nav_text: str = ""
    stat_text: str = ""
    diag_text: str = ""
    perf_text: str = ""
    pose: Optional[Tuple[int, int, int]] = None
    pose_trace: List[Tuple[int, int]] = field(default_factory=list)
    robot_state: str = "UNKNOWN"
    map_dirty: bool = True
    static_dirty: bool = True
    pose_dirty: bool = True
    status_dirty: bool = True

    def __post_init__(self) -> None:
        self.reset_rows()

    def reset_rows(self) -> None:
        self.rows = ["?" * self.width for _ in range(self.height)]
        self.pending_seen.clear()
        self.map_dirty = True
        self.static_dirty = True
        self.pose_dirty = True
        self.status_dirty = True

    def clear_scan(self) -> None:
        self.reset_rows()
        self.revision = 0
        self.state = "CLEARED"
        self.pose = None
        self.pose_trace.clear()
        self.nav_target = None
        self.nav_active = False
        self.nav_state = "IDLE"
        self.nav_index = 0
        self.nav_length = 0
        self.nav_front_mm = 0
        self.nav_blocked = False
        self.nav_text = ""
        self.stat_text = ""
        self.diag_text = ""
        self.perf_text = ""
        self.robot_state = "STOPPED"
        self.map_dirty = True
        self.static_dirty = True
        self.pose_dirty = True
        self.status_dirty = True

    def configure(self, state: str, width: int, height: int, cell_mm: int, revision: int) -> None:
        shape_changed = (width != self.width) or (height != self.height)
        new_map_started = state in {"START", "CLEAR"} or revision < self.revision
        self.state = state
        self.width = width
        self.height = height
        self.cell_mm = cell_mm
        self.revision = revision
        if shape_changed or new_map_started or len(self.rows) != self.height:
            self.reset_rows()
        self.map_dirty = True
        self.static_dirty = True
        self.status_dirty = True

    def update_row(self, y: int, revision: int, data: str) -> None:
        if not 0 <= y < self.height:
            return
        if len(data) < self.width:
            data = data.ljust(self.width, "?")
        elif len(data) > self.width:
            data = data[: self.width]

        self.rows[y] = data
        self.pending_seen.add(y)
        self.revision = revision
        self.map_dirty = True
        self.status_dirty = True
        if len(self.pending_seen) >= self.height:
            self.pending_seen.clear()

    def update_stat(self, body: str) -> None:
        self.stat_text = body
        pose_match = POSE_RE.search(body)
        if pose_match:
            self.update_pose(tuple(int(pose_match.group(i)) for i in range(1, 4)))
        state_match = POSE_STATE_RE.search(body)
        if state_match:
            self.robot_state = state_match.group(1)
        self.status_dirty = True

    def update_diag(self, body: str) -> None:
        self.diag_text = body
        self.status_dirty = True

    def update_perf(self, body: str) -> None:
        self.perf_text = body
        self.status_dirty = True

    def update_nav(self, body: str) -> None:
        self.nav_text = body
        active = NAV_ACTIVE_RE.search(body)
        state = NAV_STATE_RE.search(body)
        index = NAV_INDEX_RE.search(body)
        target = NAV_TARGET_RE.search(body)
        target_none = NAV_TARGET_NONE_RE.search(body)
        front = NAV_FRONT_RE.search(body)
        block = NAV_BLOCK_RE.search(body)

        if active:
            self.nav_active = active.group(1) == "1"
        if state:
            self.nav_state = state.group(1)
        if index:
            self.nav_index = int(index.group(1))
            self.nav_length = int(index.group(2))
        if target:
            self.nav_target = (int(target.group(1)), int(target.group(2)))
        elif target_none:
            self.nav_target = None
        if front:
            self.nav_front_mm = int(front.group(1))
        if block:
            self.nav_blocked = block.group(1) == "1"

        self.pose_dirty = True
        self.status_dirty = True

    def update_nav_event(self, line: str) -> None:
        self.nav_text = line

        run = NAV_RUN_RE.match(line)
        if run:
            self.nav_active = True
            self.nav_blocked = False
            self.nav_state = "RUN"
            self.nav_start_cell = (int(run.group(1)), int(run.group(2)))
            self.nav_goal_cell = (int(run.group(3)), int(run.group(4)))
            self.pose_trace.clear()
            self.nav_index = 0
            self.nav_length = int(run.group(5))
            self.static_dirty = True
            self.pose_dirty = True
            self.status_dirty = True
            return

        replan = NAV_REPLAN_RE.match(line)
        if replan:
            self.nav_active = True
            self.nav_blocked = False
            self.nav_state = "REPLAN"
            self.nav_start_cell = (int(replan.group(1)), int(replan.group(2)))
            self.nav_goal_cell = (int(replan.group(3)), int(replan.group(4)))
            self.nav_index = 0
            self.nav_length = int(replan.group(5))
            self.static_dirty = True
            self.pose_dirty = True
            self.status_dirty = True
            return

        leg = NAV_LEG_RE.match(line)
        if leg:
            self.nav_active = True
            self.nav_blocked = False
            self.nav_state = "LEG"
            self.nav_index = int(leg.group(1))
            self.nav_target = (int(leg.group(4)), int(leg.group(5)))
            self.pose_dirty = True
            self.status_dirty = True
            return

        blocked = NAV_BLOCKED_RE.match(line)
        if blocked:
            self.nav_active = True
            self.nav_blocked = True
            self.nav_state = "BLOCKED"
            self.nav_front_mm = int(blocked.group(1))
            self.pose_dirty = True
            self.status_dirty = True
            return

        start_cell = NAV_START_CELL_RE.match(line)
        if start_cell:
            self.nav_start_cell = (int(start_cell.group(1)), int(start_cell.group(2)))
            self.static_dirty = True
            self.status_dirty = True
            return

        goal_cell = NAV_GOAL_CELL_RE.match(line)
        if goal_cell:
            self.nav_goal_cell = (int(goal_cell.group(1)), int(goal_cell.group(2)))
            self.static_dirty = True
            self.status_dirty = True
            return

        if line.startswith("NAV DONE"):
            self.nav_active = False
            self.nav_blocked = False
            self.nav_state = "DONE"
            self.nav_target = maze_cell_center(self.nav_goal_cell)
            self.pose_dirty = True
            self.status_dirty = True
            return

        if line.startswith("NAV TARGET DONE"):
            self.nav_target = None
            self.pose_dirty = True
            self.status_dirty = True
            return

        if line.startswith("NAV STOP"):
            self.nav_active = False
            self.nav_blocked = False
            self.nav_state = "STOP"
            self.pose_dirty = True
            self.status_dirty = True
            return

        if line.startswith("NAV FAIL"):
            self.nav_active = False
            self.nav_state = "FAILED"
            self.pose_dirty = True
            self.status_dirty = True

    def update_pose(self, pose: Tuple[int, int, int], robot_state: Optional[str] = None) -> None:
        self.pose = pose
        self._append_pose_trace(pose)
        if robot_state:
            self.robot_state = robot_state
        self.pose_dirty = True
        self.status_dirty = True

    def _append_pose_trace(self, pose: Tuple[int, int, int]) -> None:
        point = (pose[0], pose[1])
        if self.pose_trace:
            last_x, last_y = self.pose_trace[-1]
            if math.hypot(point[0] - last_x, point[1] - last_y) < 30.0:
                return
        self.pose_trace.append(point)
        if len(self.pose_trace) > 2000:
            del self.pose_trace[: len(self.pose_trace) - 2000]

    def set_nav_start(self, cell: Tuple[int, int]) -> None:
        self.nav_start_cell = cell
        self.static_dirty = True
        self.status_dirty = True

    def set_nav_goal(self, cell: Tuple[int, int]) -> None:
        self.nav_goal_cell = cell
        self.static_dirty = True
        self.status_dirty = True

    def known_cells(self) -> int:
        return sum(row.count(".") + row.count("#") for row in self.rows)


def parse_map_line(line: str, model: MapModel) -> bool:
    header = MAP_HEADER_RE.match(line)
    if header:
        model.configure(
            header.group("state"),
            int(header.group("w")),
            int(header.group("h")),
            int(header.group("cell")),
            int(header.group("rev")),
        )
        return True

    row = MAP_ROW_RE.match(line)
    if row:
        model.update_row(int(row.group("y")), int(row.group("rev")), row.group("data"))
        return True

    stat = MAP_STAT_RE.match(line)
    if stat:
        model.update_stat(stat.group("body"))
        return True

    diag = MAP_DIAG_RE.match(line)
    if diag:
        model.update_diag(diag.group("body"))
        return True

    perf = MAP_PERF_RE.match(line)
    if perf:
        model.update_perf(perf.group("body"))
        return True

    nav = NAV_STAT_RE.match(line)
    if nav:
        model.update_nav(nav.group("body"))
        return True

    if line.startswith("NAV "):
        model.update_nav_event(line)
        return True

    pose = POSE_LINE_RE.match(line)
    if pose:
        model.update_pose(tuple(int(pose.group(i)) for i in range(1, 4)), pose.group(4))
        return True

    return False


class SerialBackend:
    def __init__(self, rx_queue: queue.Queue[str], log_queue: queue.Queue[str]) -> None:
        self.rx_queue = rx_queue
        self.log_queue = log_queue
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.ser = None
        self.lock = threading.Lock()
        self.port = ""
        self.baudrate = DEFAULT_BAUDRATE

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baudrate: int) -> bool:
        if serial is None:
            self.log_queue.put("[ERR] pyserial is not installed")
            return False
        self.disconnect()
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.15,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except serial.SerialException as exc:
            self.log_queue.put(f"[ERR] Cannot open {port}: {exc}")
            return False

        with self.lock:
            self.ser = ser
            self.port = port
            self.baudrate = baudrate
            self.stop_event.clear()

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        self.log_queue.put(f"[OK] Opened {port} at {baudrate} baud")
        return True

    def disconnect(self) -> None:
        self.stop_event.set()
        with self.lock:
            ser = self.ser
            self.ser = None
        if ser is not None:
            try:
                if ser.is_open:
                    ser.close()
            except serial.SerialException:
                pass
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None

    def write_command(self, cmd: str) -> bool:
        payload = command_payload(cmd)
        if not payload:
            return False
        with self.lock:
            ser = self.ser
            if ser is None or not ser.is_open:
                self.log_queue.put("[ERR] Serial is not connected")
                return False
            try:
                ser.write(payload)
                ser.flush()
            except serial.SerialException as exc:
                self.log_queue.put(f"[ERR] Write failed: {exc}")
                return False
        self.log_queue.put(f"[TX] {payload!r}")
        return True

    def _read_loop(self) -> None:
        buffer = b""
        while not self.stop_event.is_set():
            with self.lock:
                ser = self.ser
            if ser is None:
                break
            try:
                data = ser.read(2048)
            except serial.SerialException as exc:
                self.log_queue.put(f"[ERR] Serial read failed: {exc}")
                break

            if not data:
                time.sleep(0.005)
                continue

            buffer += data
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                text = raw_line.decode(errors="ignore").strip()
                if text:
                    self.rx_queue.put(text)


class StatCard(QtWidgets.QFrame):
    def __init__(self, title: str, value: str = "--") -> None:
        super().__init__()
        self.setObjectName("statCard")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(4)
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setObjectName("statTitle")
        self.value_label = QtWidgets.QLabel(value)
        self.value_label.setObjectName("statValue")
        self.value_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

    def set_value(self, value: str) -> None:
        self.value_label.setText(value)


class MapView(pg.GraphicsLayoutWidget):
    def __init__(self, model: MapModel) -> None:
        super().__init__()
        self.model = model
        self.view = self.addViewBox()
        self.view.setAspectLocked(True)
        self.setBackground("#151922")
        self.image_item = pg.ImageItem(axisOrder="row-major")
        self.view.addItem(self.image_item)
        self.static_items: List[object] = []
        self.dynamic_items: List[object] = []
        self.setMinimumSize(680, 680)

    def world_bounds(self) -> Tuple[float, float, float, float]:
        half_w = self.model.width * self.model.cell_mm / 2.0
        half_h = self.model.height * self.model.cell_mm / 2.0
        return -half_w, -half_h, half_w * 2.0, half_h * 2.0

    def update_map_image(self) -> None:
        if np is None:
            return
        arr = np.empty((self.model.height, self.model.width, 4), dtype=np.uint8)
        for y, row in enumerate(self.model.rows):
            for x, char in enumerate(row):
                if char == ".":
                    arr[y, x] = COLOR_FREE
                elif char == "#":
                    arr[y, x] = COLOR_OCCUPIED if self._has_occupied_neighbor(x, y) else COLOR_UNKNOWN
                else:
                    arr[y, x] = COLOR_UNKNOWN
        arr = np.flipud(arr)
        self.image_item.setImage(arr, autoLevels=False)
        x, y, w, h = self.world_bounds()
        self.image_item.setRect(QtCore.QRectF(x, y, w, h))
        pad = max(w, h) * 0.05
        self.view.setRange(xRange=(x - pad, x + w + pad), yRange=(y - pad, y + h + pad), padding=0)

    def _has_occupied_neighbor(self, x: int, y: int) -> bool:
        for ny in range(max(0, y - 1), min(self.model.height, y + 2)):
            for nx in range(max(0, x - 1), min(self.model.width, x + 2)):
                if nx == x and ny == y:
                    continue
                if self.model.rows[ny][nx] == "#":
                    return True
        return False

    def rebuild_static_overlay(self) -> None:
        for item in self.static_items:
            self.view.removeItem(item)
        self.static_items.clear()

        origin_x = MAZE_START_X_MM - (MAZE_START_CELL[0] + 0.5) * MAZE_CELL_MM
        origin_y = MAZE_START_Y_MM - (MAZE_START_CELL[1] + 0.5) * MAZE_CELL_MM
        max_x = origin_x + MAZE_CELLS * MAZE_CELL_MM
        max_y = origin_y + MAZE_CELLS * MAZE_CELL_MM

        for i in range(MAZE_CELLS + 1):
            x_mm = origin_x + i * MAZE_CELL_MM
            self._add_line([(x_mm, origin_y), (x_mm, max_y)], COLOR_MAZE_GRID, 1)
        for j in range(MAZE_CELLS + 1):
            y_mm = origin_y + j * MAZE_CELL_MM
            self._add_line([(origin_x, y_mm), (max_x, y_mm)], COLOR_MAZE_GRID, 1)

        self._add_cell_label("S", self.model.nav_start_cell, "#8ff0b2", origin_x, origin_y)
        self._add_cell_label("E", self.model.nav_goal_cell, "#89c2ff", origin_x, origin_y)

    def update_dynamic_overlay(self) -> None:
        for item in self.dynamic_items:
            self.view.removeItem(item)
        self.dynamic_items.clear()

        self._draw_pose_trace()
        self._draw_nav_target()
        if self.model.pose is None:
            return

        x_mm, y_mm, heading_cdeg = self.model.pose
        point = pg.ScatterPlotItem(
            [x_mm],
            [y_mm],
            size=18,
            brush=pg.mkBrush(COLOR_ROBOT),
            pen=pg.mkPen("#0b5f37", width=1),
        )
        self.view.addItem(point)
        self.dynamic_items.append(point)

        heading = math.radians(heading_cdeg / 100.0)
        line_len = 220.0
        hx = x_mm + math.cos(heading) * line_len
        hy = y_mm + math.sin(heading) * line_len
        self._add_dynamic_line([(x_mm, y_mm), (hx, hy)], "#0b5f37", 4)

    def _draw_nav_target(self) -> None:
        if self.model.nav_target is None:
            return
        tx, ty = self.model.nav_target
        if self.model.nav_state == "DONE":
            color = COLOR_TARGET_DONE
        elif self.model.nav_state == "FAILED":
            color = COLOR_TARGET_FAILED
        else:
            color = COLOR_TARGET

        target = pg.ScatterPlotItem([tx], [ty], size=22, brush=None, pen=pg.mkPen(color, width=3), symbol="o")
        self.view.addItem(target)
        self.dynamic_items.append(target)

        if self.model.pose is not None:
            x_mm, y_mm, _ = self.model.pose
            self._add_dynamic_line([(x_mm, y_mm), (tx, ty)], COLOR_NAV_LINE, 2, style=QtCore.Qt.DashLine)

    def _draw_pose_trace(self) -> None:
        if len(self.model.pose_trace) < 2:
            return
        item = pg.PlotDataItem(
            [p[0] for p in self.model.pose_trace],
            [p[1] for p in self.model.pose_trace],
            pen=pg.mkPen(COLOR_TRACE, width=2),
        )
        self.view.addItem(item)
        self.dynamic_items.append(item)

    def _add_line(self, points: List[Tuple[float, float]], color: str, width: int) -> None:
        item = pg.PlotDataItem([p[0] for p in points], [p[1] for p in points], pen=pg.mkPen(color, width=width))
        self.view.addItem(item)
        self.static_items.append(item)

    def _add_dynamic_line(
        self,
        points: List[Tuple[float, float]],
        color: str,
        width: int,
        style: QtCore.Qt.PenStyle = QtCore.Qt.SolidLine,
    ) -> None:
        item = pg.PlotDataItem([p[0] for p in points], [p[1] for p in points], pen=pg.mkPen(color, width=width, style=style))
        self.view.addItem(item)
        self.dynamic_items.append(item)

    def _add_cell_label(
        self,
        text: str,
        cell: Tuple[int, int],
        color: str,
        origin_x: float,
        origin_y: float,
    ) -> None:
        cx = origin_x + (cell[0] + 0.5) * MAZE_CELL_MM
        cy = origin_y + (cell[1] + 0.5) * MAZE_CELL_MM
        dot = pg.ScatterPlotItem([cx], [cy], size=28, brush=pg.mkBrush(color), pen=None)
        label = pg.TextItem(text=text, color="#101317", anchor=(0.5, 0.5))
        label.setPos(cx, cy)
        self.view.addItem(dot)
        self.view.addItem(label)
        self.static_items.extend([dot, label])


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, default_port: str, default_baudrate: int, auto_connect: bool) -> None:
        super().__init__()
        start_cell, goal_cell = load_viewer_settings()
        self.model = MapModel(nav_start_cell=start_cell, nav_goal_cell=goal_cell)
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.serial_backend = SerialBackend(self.rx_queue, self.log_queue)
        self.connect_thread: Optional[threading.Thread] = None
        self.connecting = False
        self.log_lines = 0

        self.setWindowTitle("STM32 Mapping Viewer - Qt")
        self.resize(1220, 820)
        self._build_ui(default_port, default_baudrate)
        self._apply_style()
        self.refresh_ports(default_port)

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(20)

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_view)
        self.refresh_timer.start(33)

        if auto_connect:
            QtCore.QTimer.singleShot(100, self.connect_serial)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.serial_backend.disconnect()
        event.accept()

    def _build_ui(self, default_port: str, default_baudrate: int) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        main_layout = QtWidgets.QHBoxLayout(root)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(12)

        left_panel = QtWidgets.QFrame()
        left_panel.setObjectName("sidePanel")
        left_panel.setFixedWidth(280)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(12, 12, 12, 12)
        left_layout.setSpacing(10)

        title = QtWidgets.QLabel("STM32 Map Console")
        title.setObjectName("appTitle")
        subtitle = QtWidgets.QLabel("Windows Qt viewer")
        subtitle.setObjectName("appSubtitle")
        left_layout.addWidget(title)
        left_layout.addWidget(subtitle)

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setCurrentText(default_port)
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(["961200", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText(str(default_baudrate))

        left_layout.addWidget(QtWidgets.QLabel("Port"))
        left_layout.addWidget(self.port_combo)
        left_layout.addWidget(QtWidgets.QLabel("Baudrate"))
        left_layout.addWidget(self.baud_combo)

        port_buttons = QtWidgets.QHBoxLayout()
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        port_buttons.addWidget(self.connect_button)
        port_buttons.addWidget(self.disconnect_button)
        left_layout.addLayout(port_buttons)

        refresh_button = QtWidgets.QPushButton("Refresh Ports")
        refresh_button.clicked.connect(lambda: self.refresh_ports(self.port_combo.currentText()))
        left_layout.addWidget(refresh_button)

        nav_box = QtWidgets.QGroupBox("Navigation Cells")
        nav_layout = QtWidgets.QGridLayout(nav_box)
        nav_layout.setHorizontalSpacing(8)
        nav_layout.setVerticalSpacing(8)
        cell_validator = QtGui.QIntValidator(0, MAZE_CELLS - 1, self)
        self.start_x = QtWidgets.QLineEdit()
        self.start_y = QtWidgets.QLineEdit()
        self.goal_x = QtWidgets.QLineEdit()
        self.goal_y = QtWidgets.QLineEdit()
        for edit in (self.start_x, self.start_y, self.goal_x, self.goal_y):
            edit.setValidator(cell_validator)
            edit.setMaxLength(1)
            edit.setFixedWidth(56)
            edit.setAlignment(QtCore.Qt.AlignCenter)
            edit.setObjectName("navCellInput")
        self.start_x.setText(str(self.model.nav_start_cell[0]))
        self.start_y.setText(str(self.model.nav_start_cell[1]))
        self.goal_x.setText(str(self.model.nav_goal_cell[0]))
        self.goal_y.setText(str(self.model.nav_goal_cell[1]))
        nav_layout.addWidget(QtWidgets.QLabel("Start X"), 0, 0)
        nav_layout.addWidget(self.start_x, 0, 1)
        nav_layout.addWidget(QtWidgets.QLabel("Y"), 0, 2)
        nav_layout.addWidget(self.start_y, 0, 3)
        nav_layout.addWidget(QtWidgets.QLabel("Goal X"), 1, 0)
        nav_layout.addWidget(self.goal_x, 1, 1)
        nav_layout.addWidget(QtWidgets.QLabel("Y"), 1, 2)
        nav_layout.addWidget(self.goal_y, 1, 3)

        set_start = QtWidgets.QPushButton("Set Start")
        set_goal = QtWidgets.QPushButton("Set Goal")
        set_start.clicked.connect(self.send_start_cell)
        set_goal.clicked.connect(self.send_goal_cell)
        nav_layout.addWidget(set_start, 2, 0, 1, 2)
        nav_layout.addWidget(set_goal, 2, 2, 1, 2)

        start_nav = QtWidgets.QPushButton("Start Navigation")
        stop_nav = QtWidgets.QPushButton("Stop Navigation")
        emergency_stop = QtWidgets.QPushButton("Emergency Stop")
        stop_mapping = QtWidgets.QPushButton("Stop Mapping")
        start_nav.setObjectName("primaryButton")
        stop_nav.setObjectName("dangerButton")
        emergency_stop.setObjectName("emergencyButton")
        stop_mapping.setObjectName("warningButton")
        start_nav.clicked.connect(lambda: self.send_command("98"))
        stop_nav.clicked.connect(self.stop_navigation)
        emergency_stop.clicked.connect(self.emergency_stop)
        stop_mapping.clicked.connect(self.stop_mapping)
        nav_layout.addWidget(start_nav, 3, 0, 1, 4)
        nav_layout.addWidget(stop_nav, 4, 0, 1, 4)
        nav_layout.addWidget(emergency_stop, 5, 0, 1, 4)
        nav_layout.addWidget(stop_mapping, 6, 0, 1, 4)
        clear_scan = QtWidgets.QPushButton("Clear Scan")
        clear_scan.clicked.connect(self.clear_scan)
        nav_layout.addWidget(clear_scan, 7, 0, 1, 4)
        left_layout.addWidget(nav_box)

        custom_box = QtWidgets.QGroupBox("Custom Command")
        custom_layout = QtWidgets.QHBoxLayout(custom_box)
        custom_layout.setContentsMargins(10, 12, 10, 10)
        custom_layout.setSpacing(8)
        self.custom_command_input = QtWidgets.QLineEdit()
        self.custom_command_input.setPlaceholderText("Bluetooth text")
        self.custom_command_input.returnPressed.connect(self.send_custom_command)
        send_custom = QtWidgets.QPushButton("Send")
        send_custom.clicked.connect(self.send_custom_command)
        custom_layout.addWidget(self.custom_command_input, 1)
        custom_layout.addWidget(send_custom)
        left_layout.addWidget(custom_box)
        left_layout.addStretch()

        center_panel = QtWidgets.QFrame()
        center_panel.setObjectName("mapPanel")
        center_layout = QtWidgets.QVBoxLayout(center_panel)
        center_layout.setContentsMargins(10, 10, 10, 10)
        center_layout.setSpacing(8)
        self.map_view = MapView(self.model)
        center_layout.addWidget(self.map_view)

        right_panel = QtWidgets.QFrame()
        right_panel.setObjectName("sidePanel")
        right_panel.setFixedWidth(300)
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(12, 12, 12, 12)
        right_layout.setSpacing(10)

        self.connection_card = StatCard("Connection", "Disconnected")
        self.robot_state_card = StatCard("Robot State", "UNKNOWN")
        self.map_card = StatCard("Map", "No data")
        self.pose_card = StatCard("Pose", "--")
        self.nav_card = StatCard("Navigation", "--")
        self.front_card = StatCard("Front Obstacle", "--")
        right_layout.addWidget(self.connection_card)
        right_layout.addWidget(self.robot_state_card)
        right_layout.addWidget(self.map_card)
        right_layout.addWidget(self.pose_card)
        right_layout.addWidget(self.nav_card)
        right_layout.addWidget(self.front_card)

        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setObjectName("logText")
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(LOG_MAX_LINES)
        right_layout.addWidget(QtWidgets.QLabel("Serial Log"))
        right_layout.addWidget(self.log_text, 1)

        main_layout.addWidget(left_panel)
        main_layout.addWidget(center_panel, 1)
        main_layout.addWidget(right_panel)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #101318;
                color: #eef2f6;
                font-family: "Segoe UI", "Microsoft YaHei UI", Arial, sans-serif;
                font-size: 13px;
            }
            #sidePanel, #mapPanel {
                background: #171b22;
                border: 1px solid #29313d;
                border-radius: 8px;
            }
            #appTitle {
                font-size: 20px;
                font-weight: 700;
                color: #ffffff;
            }
            #appSubtitle, #statTitle {
                color: #97a4b5;
            }
            QGroupBox {
                border: 1px solid #2b3440;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #b8c4d4;
            }
            QPushButton {
                background: #263140;
                border: 1px solid #3a4656;
                border-radius: 6px;
                padding: 7px 8px;
                color: #eef2f6;
            }
            QPushButton:hover {
                background: #324054;
            }
            QPushButton:pressed {
                background: #1f7a4d;
            }
            #primaryButton {
                background: #1d6f45;
                border-color: #2f9d68;
                font-weight: 600;
            }
            #primaryButton:hover {
                background: #258356;
            }
            #dangerButton {
                background: #65303a;
                border-color: #8d4653;
                font-weight: 600;
            }
            #dangerButton:hover {
                background: #7a3a47;
            }
            #emergencyButton {
                background: #9b1c31;
                border: 1px solid #dc4660;
                border-radius: 6px;
                padding: 9px 8px;
                color: #ffffff;
                font-weight: 700;
            }
            #emergencyButton:hover {
                background: #b4233a;
            }
            #emergencyButton:pressed {
                background: #6f1423;
            }
            #warningButton {
                background: #6f4e1f;
                border-color: #b7791f;
                font-weight: 600;
            }
            #warningButton:hover {
                background: #805b25;
            }
            QLineEdit, QComboBox {
                background: #0e1117;
                border: 1px solid #374151;
                border-radius: 6px;
                padding: 6px;
                color: #eef2f6;
            }
            #navCellInput {
                font-size: 16px;
                font-weight: 700;
                min-height: 28px;
                max-width: 56px;
                selection-background-color: #2f9d68;
            }
            #statCard {
                background: #0e1117;
                border: 1px solid #2b3440;
                border-radius: 8px;
            }
            #statValue {
                font-size: 15px;
                font-weight: 600;
                color: #ffffff;
            }
            #logText {
                background: #0a0d12;
                border: 1px solid #2b3440;
                border-radius: 8px;
                color: #cbd5e1;
                font-family: Consolas, "Courier New", monospace;
                font-size: 12px;
            }
            """
        )

    def refresh_ports(self, prefer: str = "") -> None:
        current = prefer or self.port_combo.currentText() or DEFAULT_PORT
        ports = []
        if list_ports is not None:
            ports = [port.device for port in list_ports.comports()]
        if current and current not in ports:
            ports.insert(0, current)
        self.port_combo.clear()
        self.port_combo.addItems(ports or [DEFAULT_PORT])
        self.port_combo.setCurrentText(current)

    def connect_serial(self) -> None:
        if self.connecting:
            self.append_log("[INFO] Connection attempt already in progress")
            return
        port = self.port_combo.currentText().strip()
        try:
            baudrate = int(self.baud_combo.currentText().strip())
        except ValueError:
            self.append_log("[ERR] Invalid baudrate")
            return

        self.connecting = True
        self.connect_button.setEnabled(False)
        self.connect_button.setText("Connecting...")
        self.append_log(f"[INFO] Opening {port} at {baudrate} baud")
        self.connect_thread = threading.Thread(
            target=self._connect_worker,
            args=(port, baudrate),
            daemon=True,
        )
        self.connect_thread.start()
        self.update_status_cards()

    def _connect_worker(self, port: str, baudrate: int) -> None:
        connected = self.serial_backend.connect(port, baudrate)
        if connected:
            self.send_saved_nav_cells()
        self.log_queue.put("__CONNECT_DONE__")

    def finish_connect_attempt(self) -> None:
        self.connecting = False
        self.connect_button.setEnabled(True)
        self.connect_button.setText("Connect")
        self.update_status_cards()

    def disconnect_serial(self) -> None:
        self.serial_backend.disconnect()
        self.append_log("[OK] Serial disconnected")
        if self.connecting:
            self.finish_connect_attempt()
        self.update_status_cards()

    def send_command(self, cmd: str) -> None:
        self.serial_backend.write_command(cmd)

    def stop_navigation(self) -> None:
        self.send_command("99")
        self.model.nav_active = False
        self.model.nav_state = "STOP"
        self.model.robot_state = "STOPPED"
        self.model.pose_dirty = True
        self.model.status_dirty = True

    def emergency_stop(self) -> None:
        self.send_command("0")
        self.model.nav_active = False
        self.model.nav_state = "STOP"
        self.model.robot_state = "STOPPED"
        self.model.pose_dirty = True
        self.model.status_dirty = True

    def stop_mapping(self) -> None:
        self.send_command("MAP STOP")
        self.model.nav_active = False
        self.model.nav_state = "STOP"
        self.model.robot_state = "STOPPED"
        self.model.state = "STOP"
        self.model.pose_dirty = True
        self.model.status_dirty = True

    def send_custom_command(self) -> None:
        cmd = self.custom_command_input.text().strip()
        if not cmd:
            return
        if self.serial_backend.write_command(cmd):
            self.custom_command_input.clear()

    def save_nav_cells(self) -> None:
        save_viewer_settings(self.model.nav_start_cell, self.model.nav_goal_cell)

    def send_saved_nav_cells(self) -> None:
        start = self.model.nav_start_cell
        goal = self.model.nav_goal_cell
        self.serial_backend.write_command(f"S{start[0]}{start[1]}")
        self.serial_backend.write_command(f"G{goal[0]}{goal[1]}")

    def sync_nav_controls_from_model(self) -> None:
        self.start_x.setText(str(self.model.nav_start_cell[0]))
        self.start_y.setText(str(self.model.nav_start_cell[1]))
        self.goal_x.setText(str(self.model.nav_goal_cell[0]))
        self.goal_y.setText(str(self.model.nav_goal_cell[1]))

    def read_cell_input(self, edit: QtWidgets.QLineEdit, fallback: int) -> int:
        text = edit.text().strip()
        try:
            value = int(text)
        except ValueError:
            value = fallback
        value = clamp_maze_cell(value, fallback)
        edit.setText(str(value))
        return value

    def send_start_cell(self) -> None:
        cell = (
            self.read_cell_input(self.start_x, self.model.nav_start_cell[0]),
            self.read_cell_input(self.start_y, self.model.nav_start_cell[1]),
        )
        self.model.set_nav_start(cell)
        self.save_nav_cells()
        self.send_command(f"S{cell[0]}{cell[1]}")

    def send_goal_cell(self) -> None:
        cell = (
            self.read_cell_input(self.goal_x, self.model.nav_goal_cell[0]),
            self.read_cell_input(self.goal_y, self.model.nav_goal_cell[1]),
        )
        self.model.set_nav_goal(cell)
        self.save_nav_cells()
        self.send_command(f"G{cell[0]}{cell[1]}")

    def clear_scan(self) -> None:
        self.model.clear_scan()
        self.append_log("[OK] View cleared")
        if self.serial_backend.is_connected():
            self.send_command("CLEAR")

    def poll_serial(self) -> None:
        processed = 0
        rows = 0
        while processed < POLL_MAX_LINES:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            parsed = parse_map_line(line, self.model)
            if line.startswith("NAV START CELL") or line.startswith("NAV GOAL CELL"):
                self.sync_nav_controls_from_model()
            if line.startswith("MAP ROW "):
                rows += 1
            elif parsed and line.startswith("NAV STAT "):
                pass
            elif parsed or not line.startswith("MAP "):
                self.append_log(f"[RX] {line}")
            processed += 1
        if rows:
            self.append_log(f"[RX] MAP ROW x{rows}")

        while True:
            try:
                message = self.log_queue.get_nowait()
            except queue.Empty:
                break
            if message == "__CONNECT_DONE__":
                self.finish_connect_attempt()
                continue
            self.append_log(message)

    def refresh_view(self) -> None:
        if self.model.map_dirty:
            self.map_view.update_map_image()
            self.model.map_dirty = False
            self.model.static_dirty = True
        if self.model.static_dirty:
            self.map_view.rebuild_static_overlay()
            self.model.static_dirty = False
            self.model.pose_dirty = True
        if self.model.pose_dirty:
            self.map_view.update_dynamic_overlay()
            self.model.pose_dirty = False
        if self.model.status_dirty:
            self.update_status_cards()
            self.model.status_dirty = False

    def update_status_cards(self) -> None:
        if self.serial_backend.is_connected():
            self.connection_card.set_value(f"{self.serial_backend.port}\n{self.serial_backend.baudrate} baud")
        else:
            self.connection_card.set_value("Disconnected")

        self.robot_state_card.set_value(self.model.robot_state.replace("_", " ").title())

        total = self.model.width * self.model.height
        self.map_card.set_value(
            f"{self.model.state} rev {self.model.revision}\n"
            f"{self.model.width}x{self.model.height}, {self.model.cell_mm} mm\n"
            f"known {self.model.known_cells()}/{total}, rows {len(self.model.pending_seen)}/{self.model.height}"
        )

        if self.model.pose is None:
            self.pose_card.set_value("--")
        else:
            x_mm, y_mm, heading_cdeg = self.model.pose
            self.pose_card.set_value(f"x {x_mm} mm\ny {y_mm} mm\nheading {heading_cdeg / 100.0:.1f} deg")

        nav_target = "--"
        if self.model.nav_target is not None:
            nav_target = f"{self.model.nav_target[0]},{self.model.nav_target[1]}"
        self.nav_card.set_value(
            f"{self.model.nav_state} active {int(self.model.nav_active)}\n"
            f"path {self.model.nav_index}/{self.model.nav_length}\n"
            f"target {nav_target}"
        )
        block_text = "BLOCKED" if self.model.nav_blocked else "clear"
        self.front_card.set_value(f"{self.model.nav_front_mm} mm\n{block_text}")

    def append_log(self, text: str) -> None:
        self.log_text.appendPlainText(text)
        self.log_lines += 1


def check_dependencies() -> bool:
    missing = []
    if not HAVE_NUMPY:
        missing.append("numpy")
    if not HAVE_SERIAL:
        missing.append("pyserial")
    if not HAVE_QT:
        missing.extend(["PySide6", "pyqtgraph"])
    if missing:
        unique = []
        for item in missing:
            if item not in unique:
                unique.append(item)
        print("[ERROR] Missing dependencies:", ", ".join(unique))
        print("Install on Windows with:")
        print("  py -m pip install PySide6 pyqtgraph numpy pyserial")
        return False
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Qt based Windows viewer for STM32 mapping telemetry.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"serial port, default {DEFAULT_PORT}")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help=f"baudrate, default {DEFAULT_BAUDRATE}")
    parser.add_argument("--no-auto-connect", action="store_true", help="open the UI without connecting immediately")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not check_dependencies():
        return 1

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("STM32 Mapping Viewer")
    window = MainWindow(args.port, args.baudrate, not args.no_auto_connect)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
