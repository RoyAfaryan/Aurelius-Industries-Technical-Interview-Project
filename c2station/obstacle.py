import math

from c2station import geo
from c2station.models import Obstacle

LatLon = tuple[float, float]
Point = tuple[float, float]

# Shortest distance from point p (obstacle) to the line segment from a to b, in meters
def distance_to_segment_m(p: Point, a: Point, b: Point) -> float:
    px, py = p
    ax, ay = a
    bx, by = b

    # Direction of the leg, from a to b
    dx, dy = bx - ax, by - ay

    # A zero-length leg (a and b are the same point): just the distance to a
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(px - ax, py - ay)

    # How far along the leg the closest point falls (0 = at a, 1 = at b)
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq

    # Clamp so the closest point can't go past either end of the leg
    t = max(0.0, min(1.0, t))

    # The closest point on the leg, and its distance to p
    cx, cy = ax + t * dx, ay + t * dy
    
    return math.hypot(px - cx, py - cy)

# True if flying straight from start to end passes through the obstacle
def leg_hits_obstacle(start: LatLon, end: LatLon, obstacle: Obstacle) -> bool:
    # Convert to local meters with the obstacles center as the origin
    a = geo.to_local_xy(obstacle.lat, obstacle.lon, *start)
    b = geo.to_local_xy(obstacle.lat, obstacle.lon, *end)
    
    return distance_to_segment_m((0.0, 0.0), a, b) < obstacle.radius_m
   
# Index of the first leg that crosses any obstacle, or None if the path is clear 
def first_blocked_leg(points: list[LatLon], obstacles: list[Obstacle]) -> int | None:
    
    for i, (start, end) in enumerate(zip(points, points[1:])):
        for obstacle in obstacles:
            if leg_hits_obstacle(start, end, obstacle):
                return i
            
    return None

# Distance from a point to the nearest obstacle's edge, or None if there are none (Negative meanst he point is in an obstacle).
def clearance_m(lat: float, lon: float, obstacles: list[Obstacle]) -> float | None:
     
    if not obstacles:    
        return None
    
    return min(geo.distance_m(lat, lon, o.lat, o.lon) - o.radius_m for o in obstacles)
    