from dataclasses import dataclass

# Dataclass object to create a waypoint with latitude, longitude, and altitude
@dataclass(frozen=True, slots=True)
class Waypoint:
    lat: float
    lon: float
    alt_m: float
    
    def __post_init__(self):
        _check_lat_lon(self.lat, self.lon)
    
# Dataclass object to create an obstacle with latitude, longitude, and radius    
@dataclass(frozen=True, slots=True)
class Obstacle:
    lat: float
    lon: float
    radius: float
    
    def __post_init__(self):
        _check_lat_lon(self.lat, self.lon)
        if self.radius <= 0:
            raise ValueError("Obstacle radius must be positive.")
        
# Dataclass object to have data fields for the latest known telemetry of the Drone within the simulation
@dataclass(slots=True)
class DroneState:
    
    # Link to the drone
    connected: bool = False
    last_heartbeat_s: float = 0.0
    
    # Drone Status
    armed: bool = False
    mode: str = ""
    
    # Drone Position
    lat: float | None = None
    lon: float | None = None
    alt_m: float | None = None
    heading_deg: float | None = None
    groundspeed_mps: float | None = None
    
    # Home Position
    home_lat: float | None = None
    home_lon: float | None = None
    
    # Health and Battery
    battery_voltage: float | None = None
    battery_percent: float | None = None
    gps_fix_type: int | None = None
    satellites: int | None = None
    
    # Mission
    current_wp: int | None = None


# Dataclass object to verify if a command to the Drone was successful and to provide a message about the result of the command
@dataclass(frozen=True, slots=True)
class CommandResult:
    success: bool
    message: str
    
        

def _check_lat_lon(lat: float, lon: float):
    if not (-90 <= lat <= 90):
        raise ValueError("Latitude must be between -90 and 90 degrees.")
    if not (-180 <= lon <= 180):
        raise ValueError("Longitude must be between -180 and 180 degrees.")