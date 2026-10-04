# --- Connection ---
DEFAULT_CONNECTION_STRING = "tcp:127.0.0.1:5762"    # change if your drone is on a different port or IP address
SOURCE_SYSTEM_ID = 254                      # must differ from Mission Planner's 255
HEARTBEAT_TIMEOUT_S = 10                    # how long to wait when first connecting
LINK_LOST_AFTER_S = 3                       # no heartbeat for this long = link lost

# --- Telemetry ---
POSITION_RATE_HZ = 5                       # how often we ask the drone to send positional data
STATUS_RATE_HZ = 1                         # how often we ask the drone to send status data

# --- Commands ---
COMMAND_ACK_TIMEOUT_S = 3                   # how long to wait for COMMAND_ACK
MISSION_UPLOAD_TIMEOUT_S = 5

# --- Flight defaults ---
DEFAULT_TAKEOFF_ALT_M = 10.0
DEFAULT_SPEED_MPS = 5.0
MIN_SPEED_MPS = 1.0
MAX_SPEED_MPS = 15.0

# --- Obstacle avoidance ---
OBSTACLE_BUFFER_M = 5.0                     # extra clearance around each obstacle
DETOUR_POINTS = 2                           # waypoints added to route around an obstacle

# --- UI ---
UI_REFRESH_MS = 100                         # 10 Hz screen updates
TRAIL_MAX_POINTS = 500                      # breadcrumb trail length cap

# --- MESSAGES ---
STATUS_LOG_MAX_LINES = 50