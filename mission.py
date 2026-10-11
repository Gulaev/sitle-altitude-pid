"""
"Climb - hold - descend" altitude mission.

AltitudeMission is only the flight scenario; all control mechanics live in the
controllers:

    GUIDED -> ARM -> short takeoff -> climb to 15 m, hold
           -> fly 10 m forward (at 15 m)
           -> descend to 10 m, hold -> LAND
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from copter import Copter
from forward_controller import ForwardFlightController
from thrust_controller import ThrustController

logger = logging.getLogger(__name__)


@dataclass
class ClimbHoldDescendProfile:
    """Parameters specific to this mission - where and how far to fly."""

    handoff_altitude_m: float = 2.0        # stock takeoff altitude before PID takes over
    cruise_altitude_m: float = 15.0
    lowered_altitude_m: float = 10.0
    hold_duration_s: float = 5.0
    forward_distance_m: float = 10.0


class AltitudeMission:
    def __init__(
        self,
        copter: Copter,
        thrust_controller: ThrustController,
        forward_controller: ForwardFlightController,
        profile: ClimbHoldDescendProfile,
    ) -> None:
        self._copter = copter
        self._thrust = thrust_controller
        self._forward = forward_controller
        self._profile = profile

    def run(self) -> None:
        """Run the full mission scenario, with a guaranteed landing attempt on failure."""
        profile = self._profile
        try:
            self._copter.connect()
            self._copter.set_mode("GUIDED")
            self._copter.arm()
            self._copter.takeoff(profile.handoff_altitude_m)

            self._thrust.reach_altitude(profile.cruise_altitude_m)
            self._thrust.hold_altitude(profile.cruise_altitude_m, profile.hold_duration_s)

            self._forward.fly_forward(profile.forward_distance_m, profile.cruise_altitude_m)

            self._thrust.reach_altitude(profile.lowered_altitude_m)
            self._thrust.hold_altitude(profile.lowered_altitude_m, profile.hold_duration_s)

            self._copter.land()
            logger.info("Mission completed successfully")
        except Exception:
            logger.exception("Error during the mission - emergency switch to LAND")
            self._safely_land()
            raise

    def _safely_land(self) -> None:
        """Best-effort landing; swallow errors so the root cause is not masked."""
        try:
            self._copter.land()
        except Exception:
            logger.exception("Failed to switch to LAND")
