
import time

from c2station.models import DroneState
from c2station import config

from pymavlink import mavutil
import threading
import queue
import logging
import collections
import copy

log = logging.getLogger(__name__)

WAITABLE_TYPES = (
    "COMMAND_ACK",
    "MISSION_REQUEST",
    "MISSION_REQUEST_INT",
    "MISSION_ACK",
)

TELEMETRY_STREAMS = {
    "GLOBAL_POSITION_INT": config.POSITION_RATE_HZ,
    "VFR_HUD": config.STATUS_RATE_HZ,
    "SYS_STATUS": config.STATUS_RATE_HZ,
    "GPS_RAW_INT": config.STATUS_RATE_HZ,
    "MISSION_CURRENT": config.STATUS_RATE_HZ,
}
 
# Class that connects to drone running in simulation and provides methods to send commands and receive telemetry data
class Link:
    
    def __init__(self, conn_str: str) -> None:
        
        # the IP address and port of the drone to connect to
        self._conn_str = conn_str
        
        # Created in connect(). None means "not connected yet".
        self._master: mavutil.mavfile | None = None
        
        # the latest known state of the drone, updated by the telemetry thread
        self._drone_state = DroneState()
        
        # a lock to protect access to the drone state, since it is updated by the telemetry thread
        self._lock = threading.Lock()
        
        self._messages: collections.deque[str] = collections.deque(maxlen=config.STATUS_LOG_MAX_LINES)
        
        # One queue per waitable message type, filled by the reader thread
        self._pending: dict[str, queue.Queue] = {msg_type: queue.Queue() for msg_type in WAITABLE_TYPES}
        
        # Background reader thread
        self._reader: threading.Thread | None = None
        
        # Flag that tells self._reader to stop. 
        self._stop = threading.Event()
        
        
    # connects to the simulated drone in ArduPilot SITL
    def connect(self, timeout_s: float = config.HEARTBEAT_TIMEOUT_S) -> None:
        
        # Connect to drone in simulated ArduPilot SITL
        try:
            self._master = mavutil.mavlink_connection(self._conn_str, source_system=config.SOURCE_SYSTEM_ID)
        except OSError as e:
            raise ConnectionError(f"Failed to connect to {self._conn_str}: {e}") from e
        
        # Wait for the first heartbeat from the drone, or timeout
        heartbeat = self._master.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            self._master.close()
            raise ConnectionError(f"No heartbeat from {self._conn_str} within {timeout_s} seconds.")
        
        # Request the drone to send telemetry streams at the desired rate
        self._request_streams()
        
        # update state
        with self._lock:
            self._drone_state.connected = True
            self._drone_state.last_heartbeat_s = time.monotonic()
            
        # Log the connection details
        log.info(f"Connected to drone with system ID {self._master.target_system} and component ID {self._master.target_component}.")   
                
    
    
    # Start the background telemetry reader thread.    
    def start(self) -> None:
        
        # Guard against calling start() before connect()
        if self._master is None:
            raise RuntimeError("Must call connect() before start().")
        
        # Guard against starting the reader thread multiple times
        if self._reader is not None:
            raise RuntimeError("Reader thread already started.")
        
        # Start the background reader thread
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_thread, daemon=True)
        self._reader.start()
    
    
    
        
    # Body of the background thread: receive, filter, dispatch
    def _reader_thread(self) -> None:
        while not self._stop.is_set():
            
            # 1. Receive a message from the drone with safety timeout.
            try:
                msg = self._master.recv_match(blocking=True, timeout=0.5)
            except OSError as e:
                log.warning("Connection lost: %s", e)
                with self._lock:
                    self._drone_state.connected = False
                break
            
            # 2. Filter out bad messages.
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            if msg.get_srcSystem() != self._master.target_system:
                continue
            
            # 3. Update the drone state
            try:
                self._update_state(msg)
            except Exception as e:
                log.exception("Failed to process %s", msg_type)
                
            # 4. Store drone messages to appropriate queue for WAITABLE messages
            if msg_type == "MISSION_REQUEST_INT":
                msg_type = "MISSION_REQUEST"
            if msg_type in self._pending:
                self._pending[msg_type].put(msg)
                
              
                
            
    # Update the drone state based on the received message.
    def _update_state(self, msg) -> None:
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return
            mode = mavutil.mode_string_v10(msg)
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            now = time.monotonic()
            with self._lock:
                self._drone_state.mode = mode
                self._drone_state.armed = armed
                self._drone_state.last_heartbeat_s = now
                
        elif msg_type == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1e3
            heading = None if msg.hdg == 65535 else msg.hdg / 100
            with self._lock:
                self._drone_state.lat = lat
                self._drone_state.lon = lon
                self._drone_state.alt = alt
                self._drone_state.heading_deg = heading
        
        elif msg_type == "VFR_HUD":
            with self._lock:
                self._drone_state.groundspeed_mps = msg.groundspeed
                
        elif msg_type == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000
            percent = msg.battery_remaining if msg.battery_remaining >= 0 else None
            with self._lock:
                self._drone_state.battery_voltage = voltage
                self._drone_state.battery_percent = percent
                
        elif msg_type == "GPS_RAW_INT":
            with self._lock:
                self._drone_state.gps_fix_type = msg.fix_type
                self._drone_state.satellites = msg.satellites_visible
                
        elif msg_type == "MISSION_CURRENT":
            with self._lock:
                self._drone_state.current_wp = msg.seq
                
        elif msg_type == "HOME_POSITION":
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            with self._lock:
                self._drone_state.home_lat = lat
                self._drone_state.home_lon = lon
                
        elif msg_type == "STATUSTEXT":
            with self._lock:
                self._messages.append(msg.text)
                
                  
    
    # UI endpoint to get the latest known state of the drone safely
    def get_state(self) -> DroneState:
        with self._lock:
            return copy.copy(self._drone_state)
        
        
    # Return the recent STATUSTEXT lines
    def recent_messages(self) -> list[str]:
        with self._lock:
            return list(self._messages)
        
    # Request the drone to send telemetry streams at the desired rate (set in config.py)
    def _request_streams(self) -> None:    
        if self._master is None:
            raise RuntimeError("Must call connect() before request_streams().")
        
        # Ask the drone to send the telemetry streams at the desired rate.
        for msg_name, rate_hz in TELEMETRY_STREAMS.items():
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_name}")
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,                      # confirmation
                msg_id,                 # param 1: which message
                1_000_000 / rate_hz,    # param 2: interval in microseconds
                0, 0, 0, 0, 0           # params 3-7: unused
            )
            log.debug("Requested %s at %d Hz", msg_name, rate_hz)
        
        # Request the drone to send the home position once
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,                                              # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,   # param 1: which message
            0, 0, 0, 0, 0, 0
        )
        log.debug("Requested HOME_POSITION")
        
    
        
        
        
        
        
        