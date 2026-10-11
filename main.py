from __future__ import annotations

import logging

from config import VehicleConfig
from copter import Copter
from flight_recorder import LiveAltitudePlotter
from forward_controller import ForwardFlightController
from mission import AltitudeMission, ClimbHoldDescendProfile
from thrust_controller import ThrustController


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    config = VehicleConfig()

    copter = Copter(
        connection_string=config.connection.connection_string,
        command_ack_timeout_s=config.connection.command_ack_timeout_s,
        message_timeout_s=config.connection.message_timeout_s,
    )

    recorder = LiveAltitudePlotter()
    thrust_controller = ThrustController(copter, config.thrust, recorder)
    forward_controller = ForwardFlightController(
        copter, config.forward, thrust_controller, recorder,
    )

    mission = AltitudeMission(
        copter, thrust_controller, forward_controller, ClimbHoldDescendProfile(),
    )
    try:
        mission.run()
    finally:
        recorder.keep_open()


if __name__ == "__main__":
    main()
