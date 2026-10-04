import logging

from pymavlink import mavutil
from typing import NamedTuple

from c2station.link import Link
from c2station import config
from c2station.models import CommandResult, DroneState, Waypoint

log = logging.getLogger(__name__)

POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

class MissionItem(NamedTuple):
    command: int
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0

class Drone:
    def __init__(self, link: Link) -> None:
        self._link = link
        
    # Open a connection to the drone without having the UI access anything in link.py
    @classmethod
    def connect(cls, conn_str: str) -> "Drone":
        link = Link(conn_str)
        try:
            link.connect()
            link.start()
        except Exception:
            link.close()
            raise
        return cls(link)

    # Close the connection to the drone
    def close(self) -> None:
        """Close the connection to the vehicle. Safe to call more than once."""
        self._link.close()
    
    # Latest telemetry snapshot
    def state(self) -> DroneState:
        return self._link.get_state()
    
    # Recent autopilot status messages, oldest first.
    def messages(self) -> list[str]:
        return self._link.recent_messages()
    
    # Send a COMMAND_LONG and wait for its ACK
    def _command(self, cmd: int, *params: float) -> CommandResult:
        # Pass only the params the command uses; the rest are padded with 0
        if len(params) > 7:
            raise ValueError("Cannot include more than 7 parameters.")
        
        padded = list(params) + [0] * (7-len(params))
        name = mavutil.mavlink.enums["MAV_CMD"][cmd].name.removeprefix("MAV_CMD_")
        
        self._link.clear_pending("COMMAND_ACK")
        self._link.mav.command_long_send(*self._link.target, cmd, 0, *padded)
        ack = self._link.wait_for("COMMAND_ACK", lambda m: m.command == cmd)
        
        if ack is None:
            log.warning("%s: no response", name)
            return CommandResult(False, f"{name}: no response")
        
        result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name.removeprefix("MAV_RESULT_")
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            log.info("%s: accepted", name)
            return CommandResult(True, f"{name}: accepted")
        
        log.warning("%s: %s", name, result)
        return CommandResult(False, f"{name}: {result}")
    
    
    # Switch flight mode. e.g. "GUIDED", "AUTO", "RTL"...
    def set_mode(self, name: str) -> CommandResult:
        return self._command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._link.mode_number(name)
        )
    
    # Arm the drone
    def arm(self) -> CommandResult:
        return self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    
    # Disarm the drone
    def disarm(self) -> CommandResult:
        return self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
    
    # Takeoff command, (default takeoff altitude is 10m)
    def takeoff(self, alt_m: float = config.DEFAULT_TAKEOFF_ALT_M) -> CommandResult:
        return self._command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)
    
    # Return to launch command
    def rtl(self) -> CommandResult:
        return self.set_mode("RTL")
    
    # Land command
    def land(self) -> CommandResult:
        return self.set_mode("LAND")
    
    # Fly to target in guided mode, switching to guided first if needed.
    def goto(self, target: Waypoint) -> CommandResult:
        if self.state().mode != "GUIDED":
            result = self.set_mode("GUIDED")
            if not result.success:
                return result
            
        self._link.mav.set_position_target_global_int_send(
            0,                                   # time_boot_ms
            *self._link.target,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, #altitude above home
            POSITION_ONLY_MASK,
            int(target.lat * 1e7),
            int(target.lon * 1e7),
            target.alt_m,
            0, 0, 0,             # vel (ignored)
            0, 0, 0,             # accel (ignored)
            0, 0,                 # yaw, yaw rate (ignored)
        )
        
        message = f"Flying to {target.lat:.6f}, {target.lon:.6f} at {target.alt_m} m"
        log.info(message)
        
        return CommandResult(True, message)
    
    # Set the speed of the drone
    def set_speed(self, mps: float) -> CommandResult:
        if not config.MIN_SPEED_MPS <= mps <= config.MAX_SPEED_MPS:
            raise ValueError(f"Speed {mps} m/s is outside {config.MIN_SPEED_MPS} to {config.MAX_SPEED_MPS} m/s")
        
        return self._command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                             mavutil.mavlink.SPEED_TYPE_GROUNDSPEED, 
                             mps, 
                             -1)
    
    # Wrap the waypoints in the items ArduPilot expects around them
    def _build_mission(self, waypoints: list[Waypoint], takeoff_alt_m: float) -> list[MissionItem]:
        s = self.state()
        home_lat = s.home_lat if s.home_lat is not None else waypoints[0].lat
        home_lon = s.home_lon if s.home_lon is not None else waypoints[0].lon
        L = mavutil.mavlink
        
        items = [MissionItem(L.MAV_CMD_NAV_WAYPOINT, home_lat, home_lon, 0)]
        items.append(MissionItem(L.MAV_CMD_NAV_TAKEOFF, alt_m=takeoff_alt_m))
        items += [MissionItem(L.MAV_CMD_NAV_WAYPOINT, wp.lat, wp.lon, wp.alt_m) for wp in waypoints]
        items.append(MissionItem(L.MAV_CMD_NAV_RETURN_TO_LAUNCH))
        return items
    
    
    # Send one mission item as MISSION_ITEM_INT, in reply to the drone's request
    def _send_mission_item(self, seq: int, item: MissionItem) -> None:
        self._link.mav.mission_item_int_send(
            *self._link.target,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # altitude above home
            item.command,
            0,                      # current (ignored by ArduPilot)
            1,                      # autocontinue to the next item
            0, 0, 0, 0,             # params 1 to 4 (hold time, radii, yaw): defaults
            int(item.lat * 1e7),    # degrees → degE7
            int(item.lon * 1e7),
            item.alt_m,
        )
    
    # Uploads waypoints as a mission: takeoff, the waypoints, then RTL
    def upload_mission(self, waypoints: list[Waypoint], takeoff_alt_m: float=config.DEFAULT_TAKEOFF_ALT_M) -> CommandResult:
        if not waypoints:
            raise ValueError("A mission needs at least one waypoint.")
        
        items = self._build_mission(waypoints, takeoff_alt_m)
        count = len(items)
        
        self._link.clear_pending("MISSION_REQUEST")
        self._link.clear_pending("MISSION_ACK")
        self._link.mav.mission_count_send(*self._link.target, count)
        log.info("Uploading mission: %d items", count)
        
        # Answer each request until the last item has been sent
        while True:
            req = self._link.wait_for("MISSION_REQUEST", timeout_s=config.MISSION_UPLOAD_TIMEOUT_S)
            if req is None:
                # The drone may have rejected the upload instead of asking for an item
                early = self._link.wait_for("MISSION_ACK", timeout_s=0.1)
                reason = _mission_result(early) if early else "timed out waiting for an item request"
                return self._failed(f"Mission upload failed: {reason}")
            if not 0 <= req.seq < count:
                return self._failed(f"Mission upload failed: drone requested invalid item {req.seq}")

            self._send_mission_item(req.seq, items[req.seq])
            if req.seq == count - 1:
                break
        
        ack = self._link.wait_for("MISSION_ACK", timeout_s=config.MISSION_UPLOAD_TIMEOUT_S)
        if ack is None:
            return self._failed("Mission upload failed: no final acknowledgement")
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return self._failed(f"Mission upload failed: {_mission_result(ack)}")

        message = f"Mission uploaded: {len(waypoints)} waypoints"
        log.info(message)
        return CommandResult(True, message)        
    
    # Arm if needed, switch to AUTO, and start the uploaded mission
    def start_mission(self) -> CommandResult:
        if not self.state().armed:
            result = self.set_mode("GUIDED")  # ArduCopter won't arm in AUTO
            if not result.success:
                return result
            result = self.arm()
            if not result.success:
                return result

        result = self.set_mode("AUTO")
        if not result.success:
            return result

        # params 0, 0 = run the whole mission (first item to last item)
        return self._command(mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
    
    # Log a failure and return it as a CommandResult
    def _failed(self, message: str) -> CommandResult:
        """Log a failure and return it as a CommandResult."""
        log.warning(message)
        return CommandResult(False, message)
    
    
# Readable name for a MISSION_ACK result
def _mission_result(ack) -> str:
    return mavutil.mavlink.enums["MAV_MISSION_RESULT"][ack.type].name.removeprefix("MAV_MISSION_")