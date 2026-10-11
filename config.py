"""
Vehicle-wide configuration shared by every mission.

Mission-specific profiles live next to their mission class, not here. This module
only holds settings that are identical regardless of the scenario.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ConnectionConfig:
    """MAVLink connection parameters."""

    connection_string: str = "udpin:127.0.0.1:14550"
    command_ack_timeout_s: float = 5.0
    message_timeout_s: float = 3.0


@dataclass
class ThrustControllerConfig:
    """Altitude control via THRUST: a PID mapping altitude error to a thrust offset around hover."""

    altitude_pid_kp: float = 0.07
    altitude_pid_ki: float = 0.02
    altitude_pid_kd: float = 0.062

    # SET_ATTITUDE_TARGET thrust-only (requires GUID_OPTIONS=8). hover_thrust measured from hold logs.
    hover_thrust: float = 0.33

    # Thrust-offset limits are ASYMMETRIC: descent is kept gentle because thrust 0
    # means free fall (motors off), so we never let the descent cut the motors.
    max_climb_thrust_delta: float = 0.1
    max_descent_thrust_delta: float = 0.003

    control_loop_hz: float = 20.0

    # "Altitude reached" criteria.
    altitude_tolerance_m: float = 0.3
    stable_duration_s: float = 1.5
    reach_timeout_s: float = 30.0


@dataclass
class ForwardFlightConfig:
    """Forward-flight-via-tilt settings (distance error -> pitch angle)."""

    pitch_kp: float = 0.08
    pitch_ki: float = 0.0
    pitch_kd: float = 0.15
    max_tilt_rad: float = 0.17

    control_loop_hz: float = 20.0
    distance_tolerance_m: float = 0.3
    stable_duration_s: float = 1.0
    reach_timeout_s: float = 30.0


@dataclass
class VehicleConfig:
    """Aggregate of the vehicle's shared settings; built once and passed to any mission."""

    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    thrust: ThrustControllerConfig = field(default_factory=ThrustControllerConfig)
    forward: ForwardFlightConfig = field(default_factory=ForwardFlightConfig)
