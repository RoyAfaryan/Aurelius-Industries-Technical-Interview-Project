"""Conversions between lat/lon and local meters.

Uses a flat-earth (equirectangular) approximation around an origin point,
which is accurate to well under a meter over the few kilometers a small
drone mission covers. Pure math: no drones, no MAVLink.
"""

import math

EARTH_RADIUS_M = 6_371_000.0


def to_local_xy(origin_lat: float, origin_lon: float, lat: float, lon: float) -> tuple[float, float]:
    """Meters east (x) and north (y) of the origin."""
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def from_local_xy(origin_lat: float, origin_lon: float, x: float, y: float) -> tuple[float, float]:
    """Lat/lon of the point x meters east and y meters north of the origin."""
    lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return lat, lon


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Straight-line ground distance between two points, in meters."""
    x, y = to_local_xy(lat1, lon1, lat2, lon2)
    return math.hypot(x, y)


def circle_points(lat: float, lon: float, radius_m: float, n: int = 48) -> list[tuple[float, float]]:
    """n lat/lon points evenly spaced on a circle around (lat, lon), for drawing it on the map."""
    return [
        from_local_xy(lat, lon, radius_m * math.cos(2 * math.pi * k / n), radius_m * math.sin(2 * math.pi * k / n))
        for k in range(n)
    ]