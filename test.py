import math
import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk
from typing import List, Optional, Tuple

import serial


PORT = "COM5"
BAUDRATE = 115200

DEFAULT_WIDTH = 80
DEFAULT_HEIGHT = 80
DEFAULT_CELL_MM = 100
CANVAS_PX = 640

COLOR_UNKNOWN = "#22252b"
COLOR_FREE = "#f3f5f7"
COLOR_OCCUPIED = "#e34b4b"
COLOR_ROBOT = "#29b36a"
COLOR_GRID = "#3a3f47"

MAP_HEADER_RE = re.compile(
    r"^MAP\s+(?P<state>\w+)\s+w=(?P<w>\d+)\s+h=(?P<h>\d+)\s+"
    r"cell=(?P<cell>\d+)mm\s+rev=(?P<rev>\d+)"
)
MAP_ROW_RE = re.compile(r"^MAP\s+ROW\s+y=(?P<y>\d+)\s+rev=(?P<rev>\d+)\s+data=(?P<data>[?.#]+)")
MAP_STAT_RE = re.compile(r"^MAP\s+STAT\s+(?P<body>.*)$")
POSE_RE = re.compile(r"\bpose=(-?\d+),(-?\d+),(-?\d+)")


@dataclass
class MapModel:
    width: int = DEFAULT_WIDTH
    height: int = DEFAULT_HEIGHT
    cell_mm: int = DEFAULT_CELL_MM
    revision: int = 0
    state: str = "IDLE"
    rows: List[str] = field(default_factory=list)
    stat_text: str = ""
    pose: Optional[Tuple[int, int, int]] = None
    dirty: bool = True

    def __post_init__(self) -> None:
        self.reset_rows()

    def reset_rows(self) -> None:
        self.rows = ["?" * self.width for _ in range(self.height)]
        self.dirty = True

    def configure(self, state: str, width: int, height: int, cell_mm: int, revision: int) -> None:
        shape_changed = (width != self.width) or (height != self.height)
        self.state = state
        self.width = width
        self.height = height
        self.cell_mm = cell_mm
        self.revision = revision
        if shape_changed or len(self.rows) != self.height:
            self.reset_rows()
        self.dirty = True

    def update_row(self, y: int, revision: int, data: str) -> None:
        if not 0 <= y < self.height:
            return

        if len(data) < self.width:
            data = data.ljust(self.width, "?")
        elif len(data) > self.width:
            data = data[: self.width]

        self.rows[y] = data
        self.revision = revision
        self.dirty = True

    def update_stat(self, body: str) -> None:
        self.stat_text = body
        pose_match = POSE_RE.search(body)
        if pose_match:
            self.pose = tuple(int(pose_match.group(i)) for i in range(1, 4))
        self.dirty = True


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

    return False


def read_loop(ser: serial.Serial, rx_queue: queue.Queue, stop_event: threading.Event) -> None:
    buffer = b""

    while not stop_event.is_set():
        try:
            data = ser.read(1024)
            if not data:
                time.sleep(0.01)
                continue

            buffer += data
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                text = raw_line.decode(errors="ignore").strip()
                if text:
                    print(f"[RX] {text}")
                    rx_queue.put(text)

        except serial.SerialException as e:
            if not stop_event.is_set():
                print(f"[ERROR] Serial read failed: {e}")
            break


def send_command(ser: serial.Serial, cmd: str) -> None:
    cmd = cmd.strip()
    if not cmd:
        return

    if cmd in ["0", "1", "2", "3"]:
        payload = cmd.encode()
    elif cmd == "91":
        payload = b"91\r\n"
    else:
        payload = (cmd + "\r\n").encode()

    ser.write(payload)
    ser.flush()
    print(f"[TX] {repr(payload)}")


class MapViewer:
    def __init__(
        self,
        root: tk.Tk,
        ser: serial.Serial,
        model: MapModel,
        rx_queue: queue.Queue,
        stop_event: threading.Event,
    ) -> None:
        self.root = root
        self.ser = ser
        self.model = model
        self.rx_queue = rx_queue
        self.stop_event = stop_event

        self.root.title("STM32 Mapping Viewer")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(root, width=CANVAS_PX, height=CANVAS_PX, bg=COLOR_UNKNOWN, highlightthickness=0)
        self.canvas.grid(row=0, column=0, columnspan=5, padx=10, pady=(10, 6), sticky="nsew")

        self.status_var = tk.StringVar(value="No map data yet. Send 91 to start mapping.")
        self.status = ttk.Label(root, textvariable=self.status_var, wraplength=CANVAS_PX)
        self.status.grid(row=1, column=0, columnspan=5, padx=10, pady=(0, 6), sticky="w")

        self.command_var = tk.StringVar(value="91")
        self.command_entry = ttk.Entry(root, textvariable=self.command_var, width=18)
        self.command_entry.grid(row=2, column=0, padx=(10, 4), pady=(0, 10), sticky="ew")
        self.command_entry.bind("<Return>", lambda _event: self.send_entry_command())

        ttk.Button(root, text="Send", command=self.send_entry_command).grid(
            row=2, column=1, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="91 Start Map", command=lambda: self.send_command_text("91")).grid(
            row=2, column=2, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="0", command=lambda: self.send_command_text("0")).grid(
            row=2, column=3, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="Close", command=self.close).grid(row=2, column=4, padx=(4, 10), pady=(0, 10), sticky="ew")

        for col in range(5):
            root.columnconfigure(col, weight=1)
        root.rowconfigure(0, weight=1)

        self.root.after(40, self.poll_serial_lines)
        self.root.after(120, self.redraw_if_needed)

    def send_entry_command(self) -> None:
        self.send_command_text(self.command_var.get())

    def send_command_text(self, cmd: str) -> None:
        try:
            send_command(self.ser, cmd)
        except serial.SerialException as e:
            print(f"[ERROR] Serial write failed: {e}")

    def poll_serial_lines(self) -> None:
        if self.stop_event.is_set():
            self.root.destroy()
            return

        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            parse_map_line(line, self.model)

        self.root.after(40, self.poll_serial_lines)

    def redraw_if_needed(self) -> None:
        if self.model.dirty:
            self.draw_map()
            self.model.dirty = False

        if not self.stop_event.is_set():
            self.root.after(120, self.redraw_if_needed)

    def draw_map(self) -> None:
        self.canvas.delete("all")
        if self.model.width <= 0 or self.model.height <= 0:
            return

        cell_px = min(CANVAS_PX / self.model.width, CANVAS_PX / self.model.height)
        map_w_px = cell_px * self.model.width
        map_h_px = cell_px * self.model.height
        ox = (CANVAS_PX - map_w_px) / 2.0
        oy = (CANVAS_PX - map_h_px) / 2.0

        for y, row in enumerate(self.model.rows):
            y0 = oy + y * cell_px
            y1 = y0 + cell_px
            for x, char in enumerate(row):
                x0 = ox + x * cell_px
                x1 = x0 + cell_px
                if char == ".":
                    color = COLOR_FREE
                elif char == "#":
                    color = COLOR_OCCUPIED
                else:
                    color = COLOR_UNKNOWN
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline=color)

        self.draw_grid_axes(ox, oy, cell_px)
        self.draw_robot_pose(ox, oy, cell_px)
        self.update_status_text()

    def draw_grid_axes(self, ox: float, oy: float, cell_px: float) -> None:
        center_x = ox + (self.model.width / 2.0) * cell_px
        center_y = oy + (self.model.height / 2.0) * cell_px
        self.canvas.create_line(center_x, oy, center_x, oy + self.model.height * cell_px, fill=COLOR_GRID)
        self.canvas.create_line(ox, center_y, ox + self.model.width * cell_px, center_y, fill=COLOR_GRID)

    def draw_robot_pose(self, ox: float, oy: float, cell_px: float) -> None:
        if self.model.pose is None:
            return

        x_mm, y_mm, heading_cdeg = self.model.pose
        cell_x = (x_mm + (self.model.width * self.model.cell_mm) / 2.0) / self.model.cell_mm
        cell_y = ((self.model.height * self.model.cell_mm) / 2.0 - y_mm) / self.model.cell_mm
        if not (0 <= cell_x < self.model.width and 0 <= cell_y < self.model.height):
            return

        cx = ox + (cell_x + 0.5) * cell_px
        cy = oy + (cell_y + 0.5) * cell_px
        radius = max(4.0, cell_px * 1.2)
        self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill=COLOR_ROBOT, outline="")

        heading_rad = math.radians(heading_cdeg / 100.0)
        line_len = max(12.0, cell_px * 3.0)
        hx = cx + math.cos(heading_rad) * line_len
        hy = cy - math.sin(heading_rad) * line_len
        self.canvas.create_line(cx, cy, hx, hy, fill="#0b5f37", width=3)

    def update_status_text(self) -> None:
        known = sum(row.count(".") + row.count("#") for row in self.model.rows)
        total = self.model.width * self.model.height
        text = (
            f"state={self.model.state} rev={self.model.revision} "
            f"size={self.model.width}x{self.model.height} cell={self.model.cell_mm}mm "
            f"known={known}/{total}"
        )
        if self.model.pose is not None:
            x_mm, y_mm, heading_cdeg = self.model.pose
            text += f" pose=({x_mm},{y_mm},{heading_cdeg / 100.0:.1f}deg)"
        if self.model.stat_text:
            text += f" | {self.model.stat_text}"
        self.status_var.set(text)

    def close(self) -> None:
        self.stop_event.set()
        self.root.destroy()


def console_input_loop(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break

        if cmd.lower() in ["exit", "quit", "q"]:
            stop_event.set()
            break
        if cmd:
            try:
                send_command(ser, cmd)
            except serial.SerialException as e:
                print(f"[ERROR] Serial write failed: {e}")
                stop_event.set()
                break


def main() -> None:
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {PORT}: {e}")
        return

    print(f"[INFO] Opened {PORT} at {BAUDRATE} baud")
    print("[INFO] Send 91 to start mapping. MAP ROW uses '?' unknown, '.' free, '#' occupied.")
    print("[INFO] You can type commands here, or use the GUI buttons. Type exit to quit.")
    print()

    rx_queue = queue.Queue()
    stop_event = threading.Event()
    model = MapModel()

    reader = threading.Thread(target=read_loop, args=(ser, rx_queue, stop_event), daemon=True)
    reader.start()
    console = threading.Thread(target=console_input_loop, args=(ser, stop_event), daemon=True)
    console.start()

    root = tk.Tk()
    MapViewer(root, ser, model, rx_queue, stop_event)

    try:
        root.mainloop()
    finally:
        stop_event.set()
        if ser.is_open:
            ser.close()
        print("\n[INFO] Serial closed")


if __name__ == "__main__":
    main()
