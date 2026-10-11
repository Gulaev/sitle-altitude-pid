"""
Forward-flight control loop driven by tilt (via SET_ATTITUDE_TARGET).

Controls two quantities at once:
    pitch (its own PID)      : distance error -> tilt angle (nose down = move forward);
    thrust (ThrustController): altitude hold, reused from the existing loop.

"Forward" is the heading (yaw) captured at the start of the phase; progress is the
projection of the position shift onto that direction.
"""

from __future__ import annotations

import logging
import math
import time

from config import ForwardFlightConfig
from copter import Copter
from flight_recorder import LiveAltitudePlotter
from pid_controller import PIDController
from thrust_controller import ThrustController

logger = logging.getLogger(__name__)


class ForwardFlightController:
    def __init__(
        self,
        copter: Copter,
        config: ForwardFlightConfig,
        thrust_controller: ThrustController,
        recorder: LiveAltitudePlotter | None = None,
    ) -> None:
        self._copter = copter
        self._config = config
        self._thrust = thrust_controller  # reused for altitude hold
        self._recorder = recorder
        self._control_period_s = 1.0 / config.control_loop_hz

        self._pitch_pid = PIDController(
            config.pitch_kp, config.pitch_ki, config.pitch_kd,
            output_min=-config.max_tilt_rad, output_max=config.max_tilt_rad,
        )

    def fly_forward(self, distance_m: float, hold_altitude_m: float) -> None:
        """Fly forward distance_m while holding altitude hold_altitude_m."""
        logger.info("Flying forward %.1f m (altitude %.1f m) ...", distance_m, hold_altitude_m)
        self._pitch_pid.reset()

        start_north, start_east, _ = self._copter.get_local_position()
        heading_rad = self._copter.get_attitude_yaw()
        forward_north, forward_east = math.cos(heading_rad), math.sin(heading_rad)

        stable_since: float | None = None
        previous_time = time.monotonic()
        deadline = previous_time + self._config.reach_timeout_s

        while True:
            now = time.monotonic()
            dt = max(now - previous_time, 1e-3)
            previous_time = now

            north, east, _ = self._copter.get_local_position()
            progress_m = (north - start_north) * forward_north + (east - start_east) * forward_east
            altitude_m = self._copter.get_relative_altitude()

            forward_tilt = self._pitch_pid.update(distance_m, progress_m, dt)
            thrust = self._thrust.compute_thrust(hold_altitude_m, altitude_m, dt)

            self._copter.send_attitude_target(
                roll_rad=0.0, pitch_rad=-forward_tilt, yaw_rad=heading_rad, thrust=thrust,
            )

            if self._recorder is not None:
                self._recorder.record(now, hold_altitude_m, altitude_m)

            if abs(distance_m - progress_m) <= self._config.distance_tolerance_m:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self._config.stable_duration_s:
                    logger.info("Distance %.1f m covered", distance_m)
                    return
            else:
                stable_since = None

            if now >= deadline:
                logger.warning("Timed out flying forward (covered %.2f m)", progress_m)
                return
            time.sleep(self._control_period_s)
