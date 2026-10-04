"""Main window of the C2 station: connection, telemetry, controls and event log.

Threading model (the one rule that keeps a Tkinter app stable):
  * Only the UI thread touches widgets.
  * Every command that talks to the drone runs on ONE worker thread, so
    clicks never freeze the window and commands run one at a time, in order.
  * The worker never touches widgets. It posts results to a queue, and the
    UI thread applies them on its next refresh (every UI_REFRESH_MS).
"""

import logging
import queue
import time
import tkinter.messagebox as messagebox
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor

import customtkinter as ctk

from c2station import config
from c2station.drone import Drone
from c2station.models import CommandResult, DroneState, Waypoint
from c2station.ui import theme

log = logging.getLogger(__name__)

FLIGHT_MODES = ["GUIDED", "AUTO", "LOITER", "BRAKE", "RTL", "LAND"]
GPS_FIX_NAMES = {0: "No GPS", 1: "No fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}

# (key, label) for each telemetry row, top to bottom
TELEMETRY_ROWS = [
    ("mode", "Mode"),
    ("armed", "State"),
    ("alt", "Altitude"),
    ("speed", "Speed"),
    ("heading", "Heading"),
    ("battery", "Battery"),
    ("gps", "GPS"),
    ("wp", "Waypoint"),
    ("position", "Position"),
]

LOG_MAX_LINES = 300
SPEED_DEBOUNCE_MS = 400


class App(ctk.CTk):
    """The operator's window. Talks to the drone only through Drone."""

    def __init__(self, default_conn: str) -> None:
        super().__init__()
        ctk.set_appearance_mode("dark")
        self.configure(fg_color=theme.BLACK)
        self.title("C2 Station")
        self.geometry("1300x880")
        self.minsize(1000, 700)
        self._fonts = theme.make_fonts()

        self._drone: Drone | None = None
        self._conn_str = ""

        self._plan: list[Waypoint] = []      # draft mission, edited by the operator
        self._plan_dirty = False             # True when the draft differs from the drone
        self._uploaded_count = 0             # waypoints in the last successful upload

        self._last_messages: list[str] = []  # autopilot messages already shown in the log
        self._telemetry_cache: dict[str, tuple[str, str]] = {}
        self._speed_job: str | None = None   # pending debounced speed command

        # One worker thread runs every command, in click order, off the UI thread
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="command")
        # Work posted back from the worker, run by the UI thread in _refresh()
        self._ui_queue: queue.Queue = queue.Queue()

        self._controls: list = []            # widgets enabled only while connected
        self._build_layout(default_conn)
        self._set_controls_enabled(False)
        self._update_plan_label()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(config.UI_REFRESH_MS, self._refresh)

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_layout(self, default_conn: str) -> None:
        f = self._fonts
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Top: connection bar
        bar = theme.panel(self)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=(10, 0))
        ctk.CTkLabel(bar, text="C2 STATION", font=f.wordmark, text_color=theme.WHITE).pack(
            side="left", padx=(16, 24), pady=10)
        self._conn_entry = theme.entry(bar, f, width=260)
        self._conn_entry.insert(0, default_conn)
        self._conn_entry.pack(side="left", pady=10)
        self._conn_entry.bind("<Return>", lambda _e: self._on_connect_clicked())
        self._connect_button = theme.button(bar, "Connect", self._on_connect_clicked, f, width=110)
        self._connect_button.pack(side="left", padx=10, pady=10)
        self._link_label = ctk.CTkLabel(bar, text="", font=f.data, corner_radius=0)
        self._link_label.pack(side="right", padx=(6, 16))
        self._link_dot = ctk.CTkLabel(bar, text="●", font=f.data)
        self._link_dot.pack(side="right")

        # Left: telemetry and flight controls
        side = ctk.CTkScrollableFrame(self, width=300, fg_color=theme.BLACK, corner_radius=0,
                                      border_width=1, border_color=theme.LINE,
                                      scrollbar_button_color=theme.LINE,
                                      scrollbar_button_hover_color=theme.EDGE)
        side.grid(row=1, column=0, sticky="nsew", padx=(10, 5), pady=10)
        self._build_telemetry(side)
        self._build_flight_controls(side)
        self._build_speed(side)

        # Right: mission toolbar above the map
        right = ctk.CTkFrame(self, fg_color="transparent", corner_radius=0)
        right.grid(row=1, column=1, sticky="nsew", padx=(5, 10), pady=10)
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        self._build_mission_toolbar(right)

        # The MapView goes in this frame in the next step
        self._map_frame = theme.panel(right)
        self._map_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        ctk.CTkLabel(self._map_frame, text="Map goes here", font=f.label, text_color=theme.EDGE).place(
            relx=0.5, rely=0.5, anchor="center")

        # Bottom: last command result and event log
        bottom = ctk.CTkFrame(self, fg_color="transparent", corner_radius=0)
        bottom.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=(0, 10))
        bottom.grid_columnconfigure(0, weight=1)
        self._status_label = ctk.CTkLabel(bottom, text="Ready", anchor="w", font=f.label,
                                          text_color=theme.GRAY, corner_radius=0)
        self._status_label.grid(row=0, column=0, sticky="w", pady=(0, 4))
        self._log_box = ctk.CTkTextbox(bottom, height=110, font=f.log, state="disabled", wrap="word",
                                       fg_color=theme.BLACK, text_color=theme.WHITE, corner_radius=0,
                                       border_width=1, border_color=theme.LINE,
                                       scrollbar_button_color=theme.LINE,
                                       scrollbar_button_hover_color=theme.EDGE)
        self._log_box.grid(row=1, column=0, sticky="ew")
        for style, (text_color, _bg) in theme.STYLES.items():
            if style != "attention":
                self._log_box.tag_config(style, foreground=text_color)

    def _section(self, parent, title: str) -> ctk.CTkFrame:
        """A titled block in the sidebar."""
        ctk.CTkLabel(parent, text=title, font=self._fonts.section, text_color=theme.WHITE, anchor="w").pack(
            fill="x", padx=8, pady=(12, 6))
        frame = ctk.CTkFrame(parent, fg_color="transparent", corner_radius=0)
        frame.pack(fill="x", padx=4)
        return frame

    def _build_telemetry(self, parent) -> None:
        frame = self._section(parent, "Telemetry")
        frame.grid_columnconfigure(1, weight=1)
        self._telemetry_labels: dict[str, ctk.CTkLabel] = {}
        for row, (key, label) in enumerate(TELEMETRY_ROWS):
            ctk.CTkLabel(frame, text=label, text_color=theme.GRAY, font=self._fonts.label,
                         anchor="w", width=72).grid(row=row, column=0, sticky="w", padx=(6, 6))
            value = ctk.CTkLabel(frame, text="--", font=self._fonts.data, anchor="w", height=26, corner_radius=0)
            value.grid(row=row, column=1, sticky="w", pady=1)
            self._telemetry_labels[key] = value

    def _build_flight_controls(self, parent) -> None:
        f = self._fonts
        frame = self._section(parent, "Flight")
        frame.grid_columnconfigure((0, 1), weight=1, uniform="flight")

        arm = theme.button(frame, "Arm", self._on_arm, f)
        disarm = theme.button(frame, "Disarm", self._on_disarm, f)
        arm.grid(row=0, column=0, sticky="ew", padx=(4, 3), pady=3)
        disarm.grid(row=0, column=1, sticky="ew", padx=(3, 4), pady=3)

        self._alt_entry = theme.entry(frame, f, justify="center")
        self._alt_entry.insert(0, f"{config.DEFAULT_TAKEOFF_ALT_M:g}")
        takeoff = theme.button(frame, "Take off (m)", self._on_takeoff, f)
        self._alt_entry.grid(row=1, column=0, sticky="ew", padx=(4, 3), pady=3)
        takeoff.grid(row=1, column=1, sticky="ew", padx=(3, 4), pady=3)

        self._mode_menu = theme.option_menu(frame, FLIGHT_MODES, f)
        set_mode = theme.button(frame, "Set mode", self._on_set_mode, f)
        self._mode_menu.grid(row=2, column=0, sticky="ew", padx=(4, 3), pady=3)
        set_mode.grid(row=2, column=1, sticky="ew", padx=(3, 4), pady=3)

        land = theme.button(frame, "Land", self._on_land, f)
        land.grid(row=3, column=0, sticky="ew", padx=(4, 3), pady=3)
        rtl = theme.button(frame, "RTL", self._on_rtl, f, primary=True)
        rtl.grid(row=3, column=1, sticky="ew", padx=(3, 4), pady=3)

        self._controls += [arm, disarm, self._alt_entry, takeoff, self._mode_menu, set_mode, land, rtl]

    def _build_speed(self, parent) -> None:
        frame = self._section(parent, "Speed")
        frame.grid_columnconfigure(0, weight=1)
        self._speed_value = ctk.CTkLabel(frame, text=f"{config.DEFAULT_SPEED_MPS:.1f} m/s",
                                         font=self._fonts.data, text_color=theme.WHITE, anchor="w")
        self._speed_value.grid(row=0, column=0, sticky="w", padx=6)
        steps = int((config.MAX_SPEED_MPS - config.MIN_SPEED_MPS) * 2)  # 0.5 m/s steps
        self._speed_slider = theme.slider(frame, from_=config.MIN_SPEED_MPS, to=config.MAX_SPEED_MPS,
                                          number_of_steps=steps, command=self._on_speed_changed)
        self._speed_slider.set(config.DEFAULT_SPEED_MPS)
        self._speed_slider.grid(row=1, column=0, sticky="ew", padx=6, pady=(6, 12))
        self._controls.append(self._speed_slider)

    def _build_mission_toolbar(self, parent) -> None:
        f = self._fonts
        bar = theme.panel(parent)
        bar.grid(row=0, column=0, sticky="ew")
        ctk.CTkLabel(bar, text="Mission", font=f.section, text_color=theme.WHITE).pack(side="left", padx=(14, 12), pady=10)
        self._plan_label = ctk.CTkLabel(bar, text="", font=f.label, corner_radius=0)
        self._plan_label.pack(side="left")

        start = theme.button(bar, "Start mission", self._on_start_mission, f, width=120)
        upload = theme.button(bar, "Upload", self._on_upload, f, width=90)
        clear = theme.button(bar, "Clear plan", self._on_clear_plan, f, width=90)
        # Temporary until the map exists: a quick way to put a plan in place
        square = theme.button(bar, "Test square", self._on_test_square, f, width=100)
        for widget in (start, upload, clear, square):  # packed right to left
            widget.pack(side="right", padx=(0, 8), pady=8)

        self._controls += [upload, start, square]

    # ------------------------------------------------------------------
    # Refresh loop (UI thread, every UI_REFRESH_MS)
    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        try:
            self._drain_ui_queue()
            state = self._drone.state() if self._drone is not None else None
            self._update_telemetry(state)
            self._update_link_label(state)
            self._update_log_from_autopilot()
        except Exception:
            log.exception("UI refresh failed")
        finally:
            # Always reschedule, so one bad refresh can't freeze the display
            self.after(config.UI_REFRESH_MS, self._refresh)

    def _drain_ui_queue(self) -> None:
        while True:
            try:
                fn, args = self._ui_queue.get_nowait()
            except queue.Empty:
                return
            fn(*args)

    def _update_telemetry(self, state: DroneState | None) -> None:
        for key, (text, style) in format_telemetry(state, self._uploaded_count).items():
            if self._telemetry_cache.get(key) != (text, style):  # only redraw what changed
                self._apply_style(self._telemetry_labels[key], text, style)
                self._telemetry_cache[key] = (text, style)

    def _update_link_label(self, state: DroneState | None) -> None:
        text, style = link_status(state, self._drone is not None)
        self._apply_style(self._link_label, text, style)
        self._link_dot.configure(text_color=theme.DOT_COLORS[style])

    def _update_log_from_autopilot(self) -> None:
        if self._drone is None:
            return
        messages = self._drone.messages()
        for text in new_messages(self._last_messages, messages):
            self._log(f"Autopilot: {text}", "quiet")
        self._last_messages = messages

    @staticmethod
    def _apply_style(label: ctk.CTkLabel, text: str, style: str) -> None:
        text_color, background = theme.STYLES[style]
        if style == "attention":
            text = f" {text} "  # breathing room inside the inverted chip
        label.configure(text=text, text_color=text_color, fg_color=background)

    # ------------------------------------------------------------------
    # Running work off the UI thread
    # ------------------------------------------------------------------

    def _post(self, fn: Callable, *args) -> None:
        """Ask the UI thread to call fn(*args) on its next refresh. Safe from any thread."""
        self._ui_queue.put((fn, args))

    def _command(self, label: str, fn: Callable, *args,
                 on_success: Callable[[], None] | None = None) -> None:
        """Run a drone command on the worker thread and report its CommandResult."""
        if self._drone is None:
            self._show_status("Not connected", "normal")
            return
        self._show_status(f"{label}...", "quiet")
        self._worker.submit(self._run_command, label, fn, args, on_success)

    def _run_command(self, label: str, fn: Callable, args: tuple,
                     on_success: Callable[[], None] | None) -> None:
        """Runs on the worker thread. Never touches widgets directly."""
        try:
            result = fn(*args)
        except Exception as e:  # report it instead of dying silently in the thread
            log.exception("%s failed", label)
            result = CommandResult(False, f"{label} failed: {e}")
        self._post(self._show_result, result)
        if result.success and on_success is not None:
            self._post(on_success)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _on_connect_clicked(self) -> None:
        if self._drone is not None:
            self._disconnect()
            return
        conn_str = self._conn_entry.get().strip()
        if not conn_str:
            self._show_status("Enter a connection string, for example tcp:127.0.0.1:5762", "normal")
            return
        self._connect_button.configure(state="disabled")
        self._show_status(f"Connecting to {conn_str}...", "quiet")
        self._worker.submit(self._connect_worker, conn_str)

    def _connect_worker(self, conn_str: str) -> None:
        """Runs on the worker thread: connect can block for several seconds."""
        try:
            drone = Drone.connect(conn_str)
        except Exception as e:
            self._post(self._on_connect_failed, conn_str, str(e))
            return
        self._post(self._on_connected, conn_str, drone)

    def _on_connected(self, conn_str: str, drone: Drone) -> None:
        self._drone = drone
        self._conn_str = conn_str
        self._last_messages = []
        self._connect_button.configure(text="Disconnect", state="normal")
        self._conn_entry.configure(state="disabled")
        self._set_controls_enabled(True)
        self._show_result(CommandResult(True, f"Connected to {conn_str}"))

    def _on_connect_failed(self, conn_str: str, reason: str) -> None:
        self._connect_button.configure(state="normal")
        self._show_result(CommandResult(False, f"Could not connect to {conn_str}: {reason}"))

    def _disconnect(self) -> None:
        drone = self._drone
        self._drone = None
        self._drone = None
        self._set_controls_enabled(False)
        self._connect_button.configure(text="Connect")
        self._conn_entry.configure(state="normal")
        if drone is not None:
            self._worker.submit(drone.close)
        self._show_status(f"Disconnected from {self._conn_str}", "quiet")
        self._log(f"Disconnected from {self._conn_str}", "quiet")

    # ------------------------------------------------------------------
    # Flight controls
    # ------------------------------------------------------------------

    def _on_arm(self) -> None:
        if messagebox.askyesno("Arm", "Arm the motors?", parent=self):
            self._command("Arm", self._drone.arm)

    def _on_disarm(self) -> None:
        self._command("Disarm", self._drone.disarm)

    def _on_takeoff(self) -> None:
        try:
            alt_m = float(self._alt_entry.get())
        except ValueError:
            self._show_status("Takeoff altitude must be a number", "normal")
            return
        if alt_m <= 0:
            self._show_status("Takeoff altitude must be above 0 m", "normal")
            return
        if messagebox.askyesno("Take off", f"Arm if needed and take off to {alt_m:g} m?", parent=self):
            self._command(f"Take off to {alt_m:g} m", launch, self._drone, alt_m)

    def _on_set_mode(self) -> None:
        mode = self._mode_menu.get()
        self._command(f"Set mode {mode}", self._drone.set_mode, mode)

    def _on_land(self) -> None:
        self._command("Land", self._drone.land)

    def _on_rtl(self) -> None:
        self._command("RTL", self._drone.rtl)

    def _on_speed_changed(self, value: float) -> None:
        """Slider moved: update the label now, send the command once it settles."""
        self._speed_value.configure(text=f"{value:.1f} m/s")
        if self._speed_job is not None:
            self.after_cancel(self._speed_job)
        self._speed_job = self.after(SPEED_DEBOUNCE_MS, self._send_speed, value)

    def _send_speed(self, value: float) -> None:
        self._speed_job = None
        self._command(f"Speed {value:.1f} m/s", self._drone.set_speed, value)

    # ------------------------------------------------------------------
    # Mission
    # ------------------------------------------------------------------

    def set_plan(self, plan: list[Waypoint]) -> None:
        """Replace the draft mission (the map will call this as waypoints are edited)."""
        self._plan = list(plan)
        self._plan_dirty = True
        self._update_plan_label()

    def _on_upload(self) -> None:
        if not self._plan:
            self._show_status("Add waypoints first", "normal")
            return
        plan = list(self._plan)  # snapshot: the draft may change while uploading

        def on_uploaded() -> None:
            self._uploaded_count = len(plan)
            self._plan_dirty = self._plan != plan
            self._update_plan_label()

        self._command(f"Upload {len(plan)} waypoints", self._drone.upload_mission, plan, on_success=on_uploaded)

    def _on_start_mission(self) -> None:
        if self._uploaded_count == 0:
            self._show_status("Upload a mission first", "normal")
            return
        warning = "\n\nThe plan on screen has changes that are NOT uploaded." if self._plan_dirty else ""
        if messagebox.askyesno("Start mission", f"Arm if needed and start the mission?{warning}", parent=self):
            self._command("Start mission", self._drone.start_mission)

    def _on_test_square(self) -> None:
        state = self._drone.state() if self._drone else None
        if state is None or state.lat is None:
            self._show_status("No position yet", "normal")
            return
        d = 0.0003  # about 33 m
        self.set_plan([
            Waypoint(state.lat + d, state.lon, 10),
            Waypoint(state.lat + d, state.lon + d, 10),
            Waypoint(state.lat, state.lon + d, 10),
        ])
        self._log("Plan set to a 3-waypoint test square", "quiet")

    def _on_clear_plan(self) -> None:
        self.set_plan([])

    def _update_plan_label(self) -> None:
        text, style = plan_status(len(self._plan), self._plan_dirty)
        self._apply_style(self._plan_label, text, style)

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------

    def _show_status(self, text: str, style: str) -> None:
        text_color, _bg = theme.STYLES[style]
        self._status_label.configure(text=text, text_color=text_color)

    def _show_result(self, result: CommandResult) -> None:
        style = "normal" if result.success else "danger"
        self._show_status(result.message, style)
        self._log(result.message, style)

    def _log(self, text: str, style: str = "normal") -> None:
        line = f"{time.strftime('%H:%M:%S')}  {text}\n"
        self._log_box.configure(state="normal")
        self._log_box.insert("end", line, style)
        excess = int(self._log_box.index("end-1c").split(".")[0]) - LOG_MAX_LINES
        if excess > 0:
            self._log_box.delete("1.0", f"{excess + 1}.0")
        self._log_box.configure(state="disabled")
        self._log_box.see("end")

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in self._controls:
            theme.set_enabled(widget, enabled)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _on_close(self) -> None:
        state = self._drone.state() if self._drone is not None else None
        if state is not None and state.armed:
            if not messagebox.askyesno(
                "Close while armed?",
                "The drone is armed. If you close the station it continues in its current mode, "
                "and its ground station failsafe may trigger.\n\nClose anyway?",
                parent=self,
            ):
                return
        if self._drone is not None:
            self._drone.close()
        self._worker.shutdown(wait=False, cancel_futures=True)
        self.destroy()


# ----------------------------------------------------------------------
# Pure helpers (no widgets, easy to unit test).
# They return a style name ("normal", "quiet", "attention", "danger"),
# and theme.STYLES decides what each one looks like.
# ----------------------------------------------------------------------

def launch(drone: Drone, alt_m: float) -> CommandResult:
    """GUIDED, arm if needed, then take off. Stops at the first failure.

    Runs on the worker thread.
    """
    state = drone.state()
    if state.mode != "GUIDED":
        result = drone.set_mode("GUIDED")
        if not result.success:
            return result
    if not state.armed:
        result = drone.arm()
        if not result.success:
            return result
    return drone.takeoff(alt_m)


def format_telemetry(s: DroneState | None, uploaded_count: int) -> dict[str, tuple[str, str]]:
    """Turn a state snapshot into (text, style) for each telemetry row."""
    if s is None:
        return {key: ("--", "quiet") for key, _ in TELEMETRY_ROWS}

    def num(value, fmt: str, unit: str) -> str:
        return "--" if value is None else f"{value:{fmt}} {unit}"

    out = {
        "mode": (s.mode or "--", "normal"),
        "armed": ("ARMED", "attention") if s.armed else ("Disarmed", "quiet"),
        "alt": (num(s.alt_m, ".1f", "m"), "normal"),
        "speed": (num(s.groundspeed_mps, ".1f", "m/s"), "normal"),
        "heading": ("--" if s.heading_deg is None else f"{s.heading_deg:03.0f}°", "normal"),
        "position": ("--" if s.lat is None else f"{s.lat:.6f}, {s.lon:.6f}", "normal"),
    }

    volts = "--" if s.battery_voltage is None else f"{s.battery_voltage:.1f} V"
    pct = s.battery_percent
    if pct is None:
        out["battery"] = (volts, "normal")
    else:
        style = "danger" if pct < 15 else "attention" if pct < 30 else "normal"
        out["battery"] = (f"{volts}   {pct}%", style)

    if s.gps_fix_type is None:
        out["gps"] = ("--", "quiet")
    else:
        fix = GPS_FIX_NAMES.get(s.gps_fix_type, str(s.gps_fix_type))
        style = "normal" if s.gps_fix_type >= 3 else "attention" if s.gps_fix_type == 2 else "danger"
        out["gps"] = (f"{fix}   {s.satellites or 0} sats", style)

    out["wp"] = (mission_progress(s.current_wp, uploaded_count), "normal")
    return out


def mission_progress(seq: int | None, uploaded_count: int) -> str:
    """Describe the current mission item in plan terms.

    Item 0 is home, 1 is takeoff, 2 to n+1 are the plan's waypoints, n+2 is RTL.
    """
    if seq is None or uploaded_count == 0 or seq == 0:
        return "--"
    if seq == 1:
        return "Takeoff"
    if seq <= uploaded_count + 1:
        return f"{seq - 1} of {uploaded_count}"
    return "RTL"


def plan_status(count: int, dirty: bool) -> tuple[str, str]:
    """Text and style for the mission plan label."""
    if count == 0:
        return "No waypoints yet", "quiet"
    noun = "waypoint" if count == 1 else "waypoints"
    if dirty:
        return f"{count} {noun}, not uploaded", "attention"
    return f"{count} {noun}, uploaded", "quiet"


def link_status(s: DroneState | None, connected: bool) -> tuple[str, str]:
    """Text and style for the link indicator."""
    if not connected or s is None:
        return "Disconnected", "quiet"
    if not s.connected:
        return "Link closed", "danger"
    age = time.monotonic() - s.last_heartbeat_s
    if age > config.LINK_LOST_AFTER_S:
        return f"Link lost {age:.0f}s", "danger"
    return "Link OK", "normal"


def new_messages(old: list[str], new: list[str]) -> list[str]:
    """Return the messages in `new` that weren't already in `old`.

    Both lists are sliding windows over the same log (oldest first), so find
    the longest tail of `old` that matches the start of `new`; the rest is new.
    """
    for k in range(min(len(old), len(new)), 0, -1):
        if old[-k:] == new[:k]:
            return new[k:]
    return new