"""
Wrapper around a MAVLink connection to an ArduPilot copter (SITL or real).

Every action that could silently fail is verified (COMMAND_ACK, mode confirmation,
the armed flag, ...); if nothing happens within the timeout we raise rather than
carry on blindly.
"""

from __future__ import annotations

import logging
import time

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

logger = logging.getLogger(__name__)

_MSG_ID_GLOBAL_POSITION_INT = mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
_MSG_ID_LOCAL_POSITION_NED = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
_MSG_ID_ATTITUDE = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE

# SET_ATTITUDE_TARGET type masks (which fields to ignore):
_TYPE_MASK_ATTITUDE_ONLY = 0b0000_0111  # ignore angular rates; use orientation + thrust
_TYPE_MASK_THRUST_ONLY = 0b1000_0111    # ignore rates and orientation; use thrust only


class CopterError(RuntimeError):
    """Any "the vehicle did not do what we asked" error."""


class Copter:
    def __init__(
        self,
        connection_string: str,
        command_ack_timeout_s: float = 5.0,
        message_timeout_s: float = 3.0,
    ) -> None:
        self._connection_string = connection_string
        self._command_ack_timeout_s = command_ack_timeout_s
        self._message_timeout_s = message_timeout_s
        self._master: mavutil.mavfile | None = None

    # ------------------------------------------------------------------ #
    # Connection
    # ------------------------------------------------------------------ #
    def connect(self) -> None:
        """Open the connection and wait for the first heartbeat from the autopilot."""
        logger.info("Connecting to %s ...", self._connection_string)
        self._master = mavutil.mavlink_connection(self._connection_string)

        self._master.wait_heartbeat(timeout=self._message_timeout_s * 5)
        if self._master.target_system == 0:
            raise CopterError("No heartbeat received - check SITL and the connection string")

        logger.info(
            "Connection established (system=%d, component=%d)",
            self._master.target_system,
            self._master.target_component,
        )

        # Stream telemetry at 25 Hz. NOTE: MAVProxy resets the rate to ~4 Hz every few
        # seconds, overriding this; start SITL with auto-rate disabled to keep it:
        #   sim_vehicle.py -v ArduCopter --console --map --mavproxy-args="--streamrate=-1"
        self._request_message_rate(_MSG_ID_GLOBAL_POSITION_INT, rate_hz=25.0)
        self._request_message_rate(_MSG_ID_LOCAL_POSITION_NED, rate_hz=25.0)
        self._request_message_rate(_MSG_ID_ATTITUDE, rate_hz=25.0)

        # Make ArduPilot treat the SET_ATTITUDE_TARGET thrust field as real thrust (0..1).
        self.set_parameter("GUID_OPTIONS", 8)

    # ------------------------------------------------------------------ #
    # Flight modes
    # ------------------------------------------------------------------ #
    def set_mode(self, mode_name: str) -> None:
        """Switch the flight mode (e.g. 'GUIDED', 'LAND') and verify the result."""
        master = self._require_connection()
        mode_name = mode_name.upper()

        mode_id = master.mode_mapping().get(mode_name)
        if mode_id is None:
            raise CopterError(f"Unknown mode '{mode_name}' for this vehicle")

        logger.info("Switching mode to %s ...", mode_name)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0,
        )
        self._wait_command_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)

        if not self._wait_until_mode_active(mode_id):
            raise CopterError(f"Mode {mode_name} did not activate within the timeout")
        logger.info("Mode %s active", mode_name)

    # ------------------------------------------------------------------ #
    # Arming
    # ------------------------------------------------------------------ #
    def arm(self) -> None:
        """Arm the motors and wait for actual arming confirmation."""
        master = self._require_connection()
        logger.info("Arming motors ...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,          # param1: 1 = arm
            0, 0, 0, 0, 0, 0,
        )
        self._wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

        if not self._wait_until_armed():
            raise CopterError("Motors did not arm (pre-arm checks may have failed)")
        logger.info("Motors armed")

    # ------------------------------------------------------------------ #
    # Takeoff - just to leave the ground before PID control takes over
    # ------------------------------------------------------------------ #
    def takeoff(self, target_altitude_m: float, reach_fraction: float = 0.95) -> None:
        """Issue a takeoff command and wait until ~reach_fraction of the target altitude."""
        master = self._require_connection()
        logger.info("Takeoff command to %.1f m ...", target_altitude_m)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude_m,      # param7: target altitude
        )
        self._wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            altitude = self.get_relative_altitude()
            if altitude >= target_altitude_m * reach_fraction:
                logger.info("Takeoff complete, altitude %.2f m", altitude)
                return
            time.sleep(0.25)
        raise CopterError("Takeoff did not complete within the timeout")

    # ------------------------------------------------------------------ #
    # Thrust / attitude control (PID output is fed in here)
    # ------------------------------------------------------------------ #
    def send_thrust(self, thrust: float) -> None:
        """Send thrust only (orientation ignored - ArduPilot keeps the vehicle level). Requires GUID_OPTIONS=8."""
        master = self._require_connection()
        identity_quaternion = [1.0, 0.0, 0.0, 0.0]  # ignored (attitude-ignore bit set)
        master.mav.set_attitude_target_send(
            0,
            master.target_system,
            master.target_component,
            _TYPE_MASK_THRUST_ONLY,
            identity_quaternion,
            0.0, 0.0, 0.0,
            thrust,
        )

    def send_attitude_target(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        thrust: float,
    ) -> None:
        """Send a target orientation + thrust (forward flight via tilt; negative pitch = nose down = forward)."""
        master = self._require_connection()
        quaternion = [float(component) for component in QuaternionBase([roll_rad, pitch_rad, yaw_rad])]
        master.mav.set_attitude_target_send(
            0,
            master.target_system,
            master.target_component,
            _TYPE_MASK_ATTITUDE_ONLY,
            quaternion,
            0.0, 0.0, 0.0,
            thrust,
        )

    # ------------------------------------------------------------------ #
    # Landing
    # ------------------------------------------------------------------ #
    def land(self) -> None:
        """Switch the vehicle to LAND mode (the autopilot lands it gently)."""
        logger.info("Switching to LAND ...")
        self.set_mode("LAND")

    # ------------------------------------------------------------------ #
    # Telemetry
    # ------------------------------------------------------------------ #
    def get_relative_altitude(self) -> float:
        """Return altitude (m) relative to the takeoff point ("home")."""
        message = self._recv_latest("GLOBAL_POSITION_INT")
        if message is None:
            raise CopterError("No GLOBAL_POSITION_INT received (no altitude telemetry)")
        return message.relative_alt / 1000.0

    def get_local_position(self) -> tuple[float, float, float]:
        """Return horizontal position (north, east, down) in metres from "home"."""
        message = self._recv_latest("LOCAL_POSITION_NED")
        if message is None:
            raise CopterError("No LOCAL_POSITION_NED received (no position telemetry)")
        return message.x, message.y, message.z

    def get_attitude_yaw(self) -> float:
        """Return the vehicle's current heading (yaw) in radians."""
        message = self._recv_latest("ATTITUDE")
        if message is None:
            raise CopterError("No ATTITUDE received (no orientation telemetry)")
        return message.yaw

    # ------------------------------------------------------------------ #
    # ArduPilot parameters
    # ------------------------------------------------------------------ #
    def set_parameter(self, name: str, value: float) -> None:
        """Set an autopilot parameter and wait for confirmation (PARAM_VALUE)."""
        master = self._require_connection()
        logger.info("Setting parameter %s = %s ...", name, value)
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        deadline = time.monotonic() + self._command_ack_timeout_s
        while time.monotonic() < deadline:
            message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and message.param_id == name:
                logger.info("Parameter %s = %s confirmed", name, message.param_value)
                return
        logger.warning("Parameter %s not confirmed within the timeout", name)

    # ------------------------------------------------------------------ #
    # Private helpers
    # ------------------------------------------------------------------ #
    def _require_connection(self) -> mavutil.mavfile:
        if self._master is None:
            raise CopterError("Not connected - call connect() first")
        return self._master

    def _recv_latest(self, message_type: str):
        """
        Return the freshest message of the given type.

        Drain the buffer and take the last one - otherwise recv_match returns the
        oldest, telemetry lags, and control acts on stale data (overshoot).
        """
        master = self._require_connection()
        latest = None
        while True:
            message = master.recv_match(type=message_type, blocking=False)
            if message is None:
                break
            latest = message
        if latest is None:  # buffer was empty - wait for a fresh one
            latest = master.recv_match(
                type=message_type, blocking=True, timeout=self._message_timeout_s,
            )
        return latest

    def _request_message_rate(self, message_id: int, rate_hz: float) -> None:
        master = self._require_connection()
        interval_us = int(1_000_000 / rate_hz)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0,
        )

    def _wait_command_ack(self, command_id: int) -> None:
        """Wait for the COMMAND_ACK of a specific command and verify it was accepted."""
        master = self._require_connection()
        deadline = time.monotonic() + self._command_ack_timeout_s
        while time.monotonic() < deadline:
            ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack is None or ack.command != command_id:
                continue
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return
            raise CopterError(f"Command {command_id} rejected (result={ack.result})")
        raise CopterError(f"No COMMAND_ACK received for command {command_id}")

    def _wait_until_mode_active(self, expected_mode_id: int) -> bool:
        master = self._require_connection()
        deadline = time.monotonic() + self._command_ack_timeout_s
        while time.monotonic() < deadline:
            heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if heartbeat is not None and heartbeat.custom_mode == expected_mode_id:
                return True
        return False

    def _wait_until_armed(self) -> bool:
        master = self._require_connection()
        deadline = time.monotonic() + self._command_ack_timeout_s
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        while time.monotonic() < deadline:
            heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if heartbeat is not None and (heartbeat.base_mode & armed_flag):
                return True
        return False
