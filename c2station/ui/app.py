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
from c2station.models import CommandResult, DroneState, Waypoint, Obstacle
from c2station.obstacle import LatLon, clearance_m, first_blocked_leg
from c2station.ui import theme
from c2station.ui.map_view import MapView

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
    ("clearance", "Clearance"),
    ("position", "Position"),
]

LOG_MAX_LINES = 300
SPEED_DEBOUNCE_MS = 400
CLEARANCE_WARNING_M = 10.0


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
        
        self._obstacles: list[Obstacle] = []  # no-fly circles placed by the operator
        self._was_inside_obstacle = False    # to log a warning only when the drone enters one

        self._last_messages: list[str] = []  # autopilot messages already shown in the log
        self._telemetry_cache: dict[str, tuple[str, str]] = {}
        self._link_cache: tuple[str, str] | None = None  # what the link indicator currently shows
        self._speed_job: str | None = None   # pending debounced speed command
        self._last_mode: str | None = None   # to notice when the vehicle leaves GUIDED

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
        # The status light and its text are separate labels so the light gets its own color.
        # Packed right to left: the text first (rightmost), then the light beside it.
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

        self._map = MapView(right, f,
                            on_add_waypoint=self._on_map_add_waypoint,
                            on_fly_here=self._on_map_fly_here,
                            on_remove_last=self._on_map_remove_last,
                            on_add_obstacle=self._on_map_add_obstacle,
                            on_clear_obstacles=self._on_map_clear_obstacles)
        self._map.grid(row=1, column=0, sticky="nsew", pady=(10, 0))

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
        rtl = theme.button(frame, "Return home", self._on_rtl, f, primary=True)
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
        upload = theme.button(bar, "Upload", self._on_upload, f, width=130)
        self._upload_button = upload
        clear = theme.button(bar, "Clear plan", self._on_clear_plan, f, width=90)
        for widget in (start, upload, clear):  # packed right to left
            widget.pack(side="right", padx=(0, 8), pady=8)

        self._controls += [upload, start]

    # ------------------------------------------------------------------
    # Refresh loop (UI thread, every UI_REFRESH_MS)
    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        try:
            self._drain_ui_queue()
            state = self._drone.state() if self._drone is not None else None
            self._update_telemetry(state)
            self._update_link_label(state)
            self._update_map(state)
            self._update_log_from_autopilot()
        except Exception:
            log.exception("UI refresh failed")
        finally:
            # Always reschedule, so one bad refresh can't freeze the display
            self.after(config.UI_REFRESH_MS, self._refresh)

    def _update_map(self, state: DroneState | None) -> None:
        self._map.update_vehicle(state)
        self._map.set_obstacles(self._obstacles)
        
        seq = state.current_wp if state is not None else None
        self._map.set_plan(self._plan, plan_progress(seq, self._uploaded_count, len(self._plan), self._plan_dirty))
        
        # A fly-here target only means something while in GUIDED: clear it once the mode moves on
        mode = state.mode if state is not None else None
        if self._last_mode == "GUIDED" and mode != "GUIDED":
            self._map.clear_target()
        self._last_mode = mode

        # Mid-mission edits only take effect when applied, so make that button say so
        in_mission = state is not None and state.armed and state.mode == "AUTO"
        upload_text = "Apply to flight" if in_mission and self._plan_dirty and self._plan else "Upload"
        if self._upload_button.cget("text") != upload_text:
            self._upload_button.configure(text=upload_text)

    def _drain_ui_queue(self) -> None:
        while True:
            try:
                fn, args = self._ui_queue.get_nowait()
            except queue.Empty:
                return
            fn(*args)

    def _update_telemetry(self, state: DroneState | None) -> None:
        rows = format_telemetry(state, self._uploaded_count)
        if state is not None:
            rows["clearance"] = clearance_status(state, self._obstacles)
        for key, (text, style) in rows.items():
            if self._telemetry_cache.get(key) != (text, style):  # only redraw what changed
                self._apply_style(self._telemetry_labels[key], text, style)
                self._telemetry_cache[key] = (text, style)

        # Live detection: say so in the log the moment the drone enters an obstacle
        inside = rows["clearance"][1] == "danger"
        if inside and not self._was_inside_obstacle:
            self._log("Warning: the drone is inside an obstacle", "danger")
        self._was_inside_obstacle = inside

    def _update_link_label(self, state: DroneState | None) -> None:
        status = link_status(state, self._drone is not None)
        if status == self._link_cache:
            return  # nothing changed: skip redrawing the labels
        self._link_cache = status
        text, style = status
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
        self._map.reset()
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

    def _read_altitude(self) -> float | None:
        """The altitude field in meters, or None (with a status message) if it isn't valid.

        Used for takeoff, new waypoints and fly-here targets.
        """
        try:
            alt_m = float(self._alt_entry.get())
        except ValueError:
            self._show_status("Altitude must be a number", "normal")
            return None
        if alt_m <= 0:
            self._show_status("Altitude must be above 0 m", "normal")
            return None
        return alt_m

    def _on_takeoff(self) -> None:
        alt_m = self._read_altitude()
        if alt_m is None:
            return
        if messagebox.askyesno("Take off", f"Arm if needed and take off to {alt_m:g} m?", parent=self):
            self._command(f"Take off to {alt_m:g} m", launch, self._drone, alt_m)

    def _on_set_mode(self) -> None:
        mode = self._mode_menu.get()
        self._command(f"Set mode {mode}", self._drone.set_mode, mode)

    def _on_land(self) -> None:
        self._command("Land", self._drone.land)

    def _on_rtl(self) -> None:
        # RTL flies straight home, so warn (but don't block) if that line crosses an obstacle
        state = self._drone.state() if self._drone is not None else None
        if state is not None and None not in (state.lat, state.lon, state.home_lat, state.home_lon):
            path_home = [(state.lat, state.lon), (state.home_lat, state.home_lon)]
            if first_blocked_leg(path_home, self._obstacles) is not None:
                if not messagebox.askyesno(
                        "Return home",
                        "The straight path home crosses an obstacle, and RTL flies straight home.\n\n"
                        "Return home anyway?", icon="warning", parent=self):
                    return
                self._log("Return home confirmed with an obstacle in the way", "danger")
        self._command("Return home", self._drone.rtl)

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
        """Replace the draft mission. The map redraws it on the next refresh."""
        self._plan = list(plan)
        self._plan_dirty = True
        self._update_plan_label()

    def _on_upload(self) -> None:
        if not self._plan:
            self._show_status("Add waypoints first", "normal")
            return
        plan = list(self._plan)  # snapshot: the draft may change while uploading

        # If a mission is being flown, keep flying from the same item after the upload
        state = self._drone.state()
        in_mission = state.armed and state.mode == "AUTO" and self._uploaded_count > 0
        resume_seq = state.current_wp if in_mission else None
        
        
        # Refuse a route whose straight legs cross an obstacle
        if state.lat is not None and state.lon is not None:
            first = 0 if resume_seq is None else min(max(resume_seq, 2), len(plan) + 1) - 2
            home = (state.home_lat, state.home_lon) if state.home_lat is not None else None
            problem = route_problem((state.lat, state.lon), plan[first:], first + 1, home, self._obstacles)
            if problem is not None:
                self._show_result(CommandResult(False, f"Upload refused: {problem}"))
                return

        def on_uploaded() -> None:
            self._uploaded_count = len(plan)
            self._plan_dirty = self._plan != plan
            self._update_plan_label()

        label = "Apply plan to flight" if resume_seq is not None else f"Upload {len(plan)} waypoints"
        self._command(label, upload_plan, self._drone, plan, resume_seq, on_success=on_uploaded)

    def _on_start_mission(self) -> None:
        if self._uploaded_count == 0:
            self._show_status("Upload a mission first", "normal")
            return
        warning = "\n\nThe plan on screen has changes that are NOT uploaded." if self._plan_dirty else ""
        if messagebox.askyesno("Start mission", f"Arm if needed and start the mission?{warning}", parent=self):
            self._command("Start mission", self._drone.start_mission)

    def _on_clear_plan(self) -> None:
        self.set_plan([])

    # ------------------------------------------------------------------
    # Map interactions
    # ------------------------------------------------------------------

    def _on_map_add_waypoint(self, lat: float, lon: float) -> None:
        """Left click on the map: append a waypoint at the altitude field's value."""
        alt_m = self._read_altitude()
        if alt_m is None:
            return
        try:
            waypoint = Waypoint(lat, lon, alt_m)
        except ValueError as e:  # e.g. a click past the edge of the world
            self._show_status(str(e), "normal")
            return
        self.set_plan(self._plan + [waypoint])
        state = self._drone.state() if self._drone is not None else None
        if state is not None and state.armed and state.mode == "AUTO":
            self._show_status("Waypoint added to the plan. Click Apply to flight to send it to the drone.", "normal")

    def _on_map_remove_last(self) -> None:
        if self._plan:
            self.set_plan(self._plan[:-1])

    def _on_map_fly_here(self, lat: float, lon: float) -> None:
        """Right click, Fly here: redirect the vehicle in GUIDED mode."""
        if self._drone is None:
            self._show_status("Connect to a vehicle first", "normal")
            return
        state = self._drone.state()
        if not state.armed or state.alt_m is None or state.alt_m < 1:
            self._show_status("Take off before using Fly here", "normal")
            return
        alt_m = self._read_altitude()
        if alt_m is None:
            return
        if first_blocked_leg([(state.lat, state.lon), (lat, lon)], self._obstacles) is not None:
            self._show_result(CommandResult(False, "Fly here refused: the straight path crosses an obstacle"))
            return
        if state.mode == "AUTO" and not messagebox.askyesno(
                "Fly here", "Leave the mission and fly to this point?", parent=self):
            return
        target = Waypoint(lat, lon, alt_m)
        self._command("Fly here", self._drone.goto, target,
                      on_success=lambda: self._map.set_target(lat, lon))



    def _on_map_add_obstacle(self, lat: float, lon: float) -> None:
        """Right click, Add obstacle here: a no-fly circle of OBSTACLE_RADIUS_M."""
        try:
            obstacle = Obstacle(lat, lon, config.OBSTACLE_RADIUS_M)
        except ValueError as e:  # e.g. a click past the edge of the world
            self._show_status(str(e), "normal")
            return
        self._obstacles.append(obstacle)
        self._log(f"Obstacle added at {lat:.6f}, {lon:.6f} ({obstacle.radius_m:g} m radius)", "quiet")



    def _on_map_clear_obstacles(self) -> None:
        if self._obstacles:
            self._obstacles = []
            self._log("Obstacles cleared", "quiet")
            
            
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


def upload_plan(drone: Drone, plan: list[Waypoint], resume_seq: int | None) -> CommandResult:
    """Upload the plan; if a mission was being flown, carry on from the same item.

    Runs on the worker thread. Waypoint i is mission item i + 2, so waypoints
    added to the end never change the item the vehicle is flying to.
    """
    result = drone.upload_mission(plan)
    if not result.success or resume_seq is None:
        return result
    last_seq = len(plan) + 1  # the final waypoint (the RTL item comes after it)
    resumed = drone.set_current_item(min(max(resume_seq, 2), last_seq))
    if not resumed.success:
        return resumed
    return CommandResult(True, f"Mission updated in flight: {len(plan)} waypoints")


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

def clearance_status(s: DroneState, obstacles: list[Obstacle]) -> tuple[str, str]:
    """Text and style for the Clearance row: distance from the drone to the nearest obstacle's edge."""
    if s.lat is None or s.lon is None:
        return "--", "quiet"
    clearance = clearance_m(s.lat, s.lon, obstacles)
    if clearance is None:
        return "--", "quiet"
    if clearance < 0:
        return "Inside obstacle", "danger"
    if clearance < CLEARANCE_WARNING_M:
        return f"{clearance:.0f} m", "attention"
    return f"{clearance:.0f} m", "normal"


def route_problem(drone: LatLon, waypoints: list[Waypoint], first_number: int,
                  home: LatLon | None, obstacles: list[Obstacle]) -> str | None:
    """Why flying drone -> waypoints -> home would cross an obstacle, or None if it's clear.

    first_number is the plan number of waypoints[0]: 1, unless resuming mid-mission.
    home is the final leg (the mission ends with RTL); None if home isn't known yet.
    """
    points = [drone] + [(wp.lat, wp.lon) for wp in waypoints]
    names = ["the drone"] + [f"waypoint {first_number + i}" for i in range(len(waypoints))]
    if home is not None:
        points.append(home)
        names.append("home")
    leg = first_blocked_leg(points, obstacles)
    if leg is None:
        return None
    return f"the leg from {names[leg]} to {names[leg + 1]} crosses an obstacle"

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
    return "Return home"


def plan_progress(seq: int | None, uploaded_count: int, plan_len: int, dirty: bool) -> int | None:
    """Index of the plan waypoint being flown to, plan_len when all are done, or None if unknown.

    Only meaningful when the plan on screen is the plan on the vehicle.
    Mission item 0 is home and 1 is takeoff, so waypoint i is item i + 2.
    """
    if dirty or uploaded_count == 0 or uploaded_count != plan_len or seq is None or seq < 2:
        return None
    return min(seq - 2, uploaded_count)


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