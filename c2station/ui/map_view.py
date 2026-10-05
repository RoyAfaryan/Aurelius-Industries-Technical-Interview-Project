"""The map: vehicle, trail, home, mission plan and fly-here target.

MapView knows nothing about drones or MAVLink. The app hands it plain data
(DroneState, a list of Waypoints) and the map reports clicks back through
callbacks. Like every widget, it must only be used from the UI thread.

Performance notes:
  * Everything is created once and then moved, never rebuilt per frame.
  * The trail grows only when the vehicle has moved TRAIL_MIN_STEP_M.
  * The plan is redrawn only when it actually changes.
  * Rotated vehicle icons are cached per HEADING_STEP_DEG.
"""

from collections.abc import Callable

import customtkinter as ctk
import tkintermapview
from PIL import Image, ImageDraw, ImageFont, ImageTk

from c2station import geo
from c2station.models import DroneState, Waypoint, Obstacle
from c2station.ui import theme

# Esri World Imagery: satellite tiles, no API key needed (attribution required)
TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
TILE_MAX_ZOOM = 19
TILE_ATTRIBUTION = "Tiles © Esri"

TRAIL_COLOR = "#BFBFBF"    # light gray: visible on imagery, quieter than the white plan path
VEHICLE_ZOOM = 18          # zoom level when the map first finds the vehicle
TRAIL_MIN_STEP_M = 1.0     # add a trail point only after moving this far
TRAIL_MAX_POINTS = 600     # older trail points are dropped
HEADING_STEP_DEG = 5       # vehicle icon is re-rotated in steps of this size


class MapView(ctk.CTkFrame):
    """Map panel with its own overlay controls."""

    def __init__(self, parent, fonts: theme.Fonts, *,
                 on_add_waypoint: Callable[[float, float], None],
                 on_fly_here: Callable[[float, float], None],
                 on_remove_last: Callable[[], None],
                 on_add_obstacle: Callable[[float, float], None],
                 on_clear_obstacles: Callable[[], None],
                 ) -> None:
        
        super().__init__(parent, fg_color=theme.BLACK, corner_radius=0, border_width=1, border_color=theme.LINE)
        self._fonts = fonts

        self._map = tkintermapview.TkinterMapView(self, corner_radius=0, bg_color=theme.BLACK)
        self._map.pack(fill="both", expand=True, padx=1, pady=1)
        self._map.canvas.configure(bg=theme.BLACK)  # what shows while tiles load, or offline
        # Zoom first, then position: set_zoom keeps the current center, and
        # doing it in the other order can leave the view at the wrong longitude
        self._map.set_zoom(2)
        self._map.set_position(20.0, 0.0)

        # Left click (without dragging) adds a waypoint; right click opens a menu
        self._map.add_left_click_map_command(lambda pos: on_add_waypoint(pos[0], pos[1]))
        self._map.add_right_click_menu_command("Fly here", lambda pos: on_fly_here(pos[0], pos[1]), pass_coords=True)
        self._map.add_right_click_menu_command("Remove last waypoint", on_remove_last)
        self._map.add_right_click_menu_command("Add obstacle here", lambda pos: on_add_obstacle(pos[0], pos[1]),
                                               pass_coords=True)
        self._map.add_right_click_menu_command("Clear obstacles", on_clear_obstacles)

        self._icons = _IconFactory(fonts)

        self._vehicle_marker = None
        self._vehicle_icon_key: int | None = None
        self._last_vehicle_pos: tuple[float, float] | None = None
        self._centered = False

        self._home: tuple[float, float] | None = None
        self._home_marker = None

        self._trail: list[tuple[float, float]] = []
        self._trail_path = None

        self._plan_key = None              # what the plan drawing currently shows
        self._plan_markers: list = []
        self._plan_path = None

        self._target_marker = None
        
        self._obstacles_key = None         # what the obstacle drawing currently shows
        self._obstacle_shapes: list = []

        self._map.set_tile_server(TILE_URL, max_zoom=TILE_MAX_ZOOM)
        self._build_overlay()

    # ------------------------------------------------------------------
    # Overlay controls
    # ------------------------------------------------------------------

    def _build_overlay(self) -> None:
        f = self._fonts
        center = theme.button(self, "Center on drone", self.center_on_vehicle, f, width=130)
        center.place(relx=1.0, x=-12, y=12, anchor="ne")

        attribution = ctk.CTkLabel(self, text=f" {TILE_ATTRIBUTION} ", font=f.label, text_color=theme.GRAY,
                                   fg_color=theme.BLACK, corner_radius=0)
        attribution.place(relx=1.0, rely=1.0, x=-2, y=-2, anchor="se")

        hint = ctk.CTkLabel(self, text="Click to add a waypoint. Right-click to fly here or add an obstacle.",
                            font=f.label, text_color=theme.GRAY, fg_color=theme.BLACK, corner_radius=0)
        hint.place(relx=0.0, rely=1.0, x=2, y=-2, anchor="sw")

    def center_on_vehicle(self) -> None:
        if self._last_vehicle_pos is not None:
            self._map.set_position(*self._last_vehicle_pos)

    # ------------------------------------------------------------------
    # Vehicle, trail and home (called every refresh)
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Forget the previous vehicle: call when a new connection starts."""
        self.update_vehicle(None)
        self._trail = []
        if self._trail_path is not None:
            self._trail_path.delete()
            self._trail_path = None
        if self._home_marker is not None:
            self._home_marker.delete()
            self._home_marker = None
        self._home = None
        self.clear_target()
        self._centered = False

    def update_vehicle(self, state: DroneState | None) -> None:
        if state is None or state.lat is None or state.lon is None:
            if self._vehicle_marker is not None:
                self._vehicle_marker.delete()
                self._vehicle_marker = None
                self._vehicle_icon_key = None
            return

        pos = (state.lat, state.lon)
        icon_key = _heading_bucket(state.heading_deg)
        if self._vehicle_marker is None:
            self._vehicle_marker = self._map.set_marker(*pos, icon=self._icons.vehicle(icon_key))
            self._vehicle_icon_key = icon_key
        else:
            if pos != self._last_vehicle_pos:
                self._vehicle_marker.set_position(*pos)
            if icon_key != self._vehicle_icon_key:
                self._vehicle_marker.change_icon(self._icons.vehicle(icon_key))
                self._vehicle_icon_key = icon_key
        self._last_vehicle_pos = pos

        if not self._centered:  # first fix after connecting: bring the vehicle into view
            self._map.set_zoom(VEHICLE_ZOOM)
            self._map.set_position(*pos)
            self._centered = True

        self._update_trail(pos)
        self._update_home(state)

    def _update_trail(self, pos: tuple[float, float]) -> None:
        if self._trail and geo.distance_m(*self._trail[-1], *pos) < TRAIL_MIN_STEP_M:
            return
        self._trail.append(pos)
        self._trail = self._trail[-TRAIL_MAX_POINTS:]
        if len(self._trail) < 2:  # a path needs at least two points
            return
        if self._trail_path is None:
            self._trail_path = self._map.set_path(list(self._trail), color=TRAIL_COLOR, width=2)
        else:
            self._trail_path.set_position_list(list(self._trail))

    def _update_home(self, state: DroneState) -> None:
        if state.home_lat is None or state.home_lon is None:
            return
        home = (state.home_lat, state.home_lon)
        if home == self._home:
            return
        self._home = home
        if self._home_marker is None:
            self._home_marker = self._map.set_marker(*home, icon=self._icons.home())
        else:
            self._home_marker.set_position(*home)
        self._plan_key = None  # the plan path starts at home, so redraw it

    # ------------------------------------------------------------------
    # Mission plan
    # ------------------------------------------------------------------

    def set_plan(self, plan: list[Waypoint], progress: int | None) -> None:
        """Draw the plan. progress: index of the waypoint being flown to (len(plan) when done)."""
        key = (tuple(plan), progress, self._home)
        if key == self._plan_key:
            return
        self._plan_key = key

        for marker in self._plan_markers:
            marker.delete()
        self._plan_markers = []
        if self._plan_path is not None:
            self._plan_path.delete()
            self._plan_path = None

        for i, wp in enumerate(plan):
            if progress is None or i > progress:
                status = "pending"
            elif i == progress:
                status = "current"
            else:
                status = "done"
            self._plan_markers.append(self._map.set_marker(wp.lat, wp.lon, icon=self._icons.waypoint(i + 1, status)))

        points = ([self._home] if self._home is not None else []) + [(wp.lat, wp.lon) for wp in plan]
        if len(points) >= 2:
            self._plan_path = self._map.set_path(points, color=theme.WHITE, width=2)

        self._raise_vehicle()

    # ------------------------------------------------------------------
    # Obstacles
    # ------------------------------------------------------------------

    def set_obstacles(self, obstacles: list[Obstacle]) -> None:
        """Draw each obstacle as a red circle. Redrawn only when the list changes."""
        key = tuple(obstacles)
        if key == self._obstacles_key:
            return
        self._obstacles_key = key

        for shape in self._obstacle_shapes:
            shape.delete()
        self._obstacle_shapes = [
            self._map.set_polygon(geo.circle_points(o.lat, o.lon, o.radius_m),
                                  outline_color=theme.DANGER, fill_color=None, border_width=2)
            for o in obstacles
        ]
        
    # ------------------------------------------------------------------
    # Fly-here target
    # ------------------------------------------------------------------

    def set_target(self, lat: float, lon: float) -> None:
        if self._target_marker is None:
            self._target_marker = self._map.set_marker(lat, lon, icon=self._icons.target())
        else:
            self._target_marker.set_position(lat, lon)
        self._raise_vehicle()

    def clear_target(self) -> None:
        if self._target_marker is not None:
            self._target_marker.delete()
            self._target_marker = None

    def _raise_vehicle(self) -> None:
        """Keep the vehicle drawn above waypoints and targets."""
        if self._vehicle_marker is not None and self._vehicle_marker.canvas_icon is not None:
            self._map.canvas.tag_raise(self._vehicle_marker.canvas_icon)


def _heading_bucket(heading_deg: float | None) -> int:
    if heading_deg is None:
        return 0
    return int(round(heading_deg / HEADING_STEP_DEG) * HEADING_STEP_DEG) % 360


class _IconFactory:
    """Draws map icons with Pillow, in the theme's colors, and caches them.

    The cache also keeps a reference to every PhotoImage; without one, Tkinter
    garbage-collects the image and the marker silently goes blank.
    """

    def __init__(self, fonts: theme.Fonts) -> None:
        self._cache: dict = {}
        try:
            self._font = ImageFont.load_default(size=12)
        except TypeError:  # Pillow older than 10.1
            self._font = ImageFont.load_default()

    def vehicle(self, heading_bucket: int):
        key = ("vehicle", heading_bucket)
        if key not in self._cache:
            size = 34
            img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img)
            arrow = [(17, 3), (29, 30), (17, 23), (5, 30)]  # points north
            draw.polygon(arrow, fill=theme.WHITE, outline=theme.BLACK, width=2)
            # Heading is clockwise from north; Pillow rotates counterclockwise
            img = img.rotate(-heading_bucket, resample=Image.BICUBIC)
            self._cache[key] = ImageTk.PhotoImage(img)
        return self._cache[key]

    def waypoint(self, number: int, status: str):
        key = ("waypoint", number, status)
        if key not in self._cache:
            fill, edge, text = {
                "pending": (theme.BLACK, theme.WHITE, theme.WHITE),
                "current": (theme.WHITE, theme.WHITE, theme.BLACK),  # inverted: needs attention
                "done": (theme.BLACK, theme.EDGE, theme.GRAY),
            }[status]
            self._cache[key] = self._square(str(number), fill, edge, text)
        return self._cache[key]

    def home(self):
        if "home" not in self._cache:
            self._cache["home"] = self._square("H", theme.BLACK, theme.GRAY, theme.WHITE)
        return self._cache["home"]

    def target(self):
        if "target" not in self._cache:
            size = 30
            img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img)
            draw.ellipse((4, 4, 26, 26), outline=theme.WHITE, width=2)
            for a, b in (((15, 0), (15, 9)), ((15, 21), (15, 30)), ((0, 15), (9, 15)), ((21, 15), (30, 15))):
                draw.line((a, b), fill=theme.WHITE, width=2)
            self._cache["target"] = ImageTk.PhotoImage(img)
        return self._cache["target"]

    def _square(self, label: str, fill: str, edge: str, text: str):
        size = 22
        img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.rectangle((1, 1, size - 2, size - 2), fill=fill, outline=edge, width=2)
        left, top, right, bottom = draw.textbbox((0, 0), label, font=self._font)
        draw.text(((size - (right - left)) / 2 - left, (size - (bottom - top)) / 2 - top), label,
                  fill=text, font=self._font)
        return ImageTk.PhotoImage(img)