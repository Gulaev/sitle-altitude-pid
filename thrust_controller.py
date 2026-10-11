"""
Altitude control loop driven by THRUST (via SET_ATTITUDE_TARGET, thrust-only).

The mission uses two intents:
    reach_altitude(target)         - climb to an altitude and stabilise;
    hold_altitude(target, seconds) - hold an altitude for a number of seconds.

The thrust calculation is exposed via compute_thrust() so ForwardFlightController
can reuse it (it adds its own pitch to the same SET_ATTITUDE_TARGET command).
"""

from __future__ import annotations

import logging
import time

from config import ThrustControllerConfig
from copter import Copter
from flight_recorder import LiveAltitudePlotter
from pid_controller import PIDController

logger = logging.getLogger(__name__)


class ThrustController:
    def __init__(
        self,
        copter: Copter,
        config: ThrustControllerConfig,
        recorder: LiveAltitudePlotter | None = None,
    ) -> None:
        self._copter = copter
        self._config = config
        self._recorder = recorder
        self._control_period_s = 1.0 / config.control_loop_hz

        # Altitude error -> thrust offset around hover. Limits are asymmetric
        # (generous climb, gentle descent) so descent does not become free fall.
        self._pid = PIDController(
            kp=config.altitude_pid_kp,
            ki=config.altitude_pid_ki,
            kd=config.altitude_pid_kd,
            output_min=-config.max_descent_thrust_delta,
            output_max=config.max_climb_thrust_delta,
        )

    def compute_thrust(self, target_altitude_m: float, current_altitude_m: float, dt: float) -> float:
        """Altitude PID -> thrust (0..1): hover + offset, clamped to limits."""
        thrust_delta = self._pid.update(target_altitude_m, current_altitude_m, dt)
        return _clamp(self._config.hover_thrust + thrust_delta, 0.0, 1.0)

    def reach_altitude(self, target_altitude_m: float) -> None:
        """Drive the vehicle to the target altitude until it holds steady within tolerance."""
        logger.info("Thrust loop: climbing to %.1f m ...", target_altitude_m)
        self._pid.reset()

        stable_since: float | None = None
        previous_time = time.monotonic()
        deadline = previous_time + self._config.reach_timeout_s

        while True:
            now = time.monotonic()
            dt = max(now - previous_time, 1e-3)
            previous_time = now

            altitude = self._step(target_altitude_m, dt)

            if abs(target_altitude_m - altitude) <= self._config.altitude_tolerance_m:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self._config.stable_duration_s:
                    logger.info("Altitude %.1f m reached (stable)", target_altitude_m)
                    return
            else:
                stable_since = None

            if now >= deadline:
                logger.warning(
                    "Timed out reaching %.1f m (current %.2f m) - moving on",
                    target_altitude_m, altitude,
                )
                return
            time.sleep(self._control_period_s)

    def hold_altitude(self, target_altitude_m: float, duration_s: float) -> None:
        """Hold the given altitude for duration_s seconds, continuing thrust control."""
        logger.info("Thrust loop: holding %.1f m for %.1f s ...", target_altitude_m, duration_s)

        previous_time = time.monotonic()
        deadline = previous_time + duration_s

        while time.monotonic() < deadline:
            now = time.monotonic()
            dt = max(now - previous_time, 1e-3)
            previous_time = now

            self._step(target_altitude_m, dt)
            time.sleep(self._control_period_s)

    def _step(self, target_altitude_m: float, dt: float) -> float:
        """Measure altitude -> thrust -> command (+record). Returns the current altitude."""
        altitude = self._copter.get_relative_altitude()
        thrust = self.compute_thrust(target_altitude_m, altitude, dt)
        self._copter.send_thrust(thrust)

        if self._recorder is not None:
            self._recorder.record(time.monotonic(), target_altitude_m, altitude)
        return altitude


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
