"""
Real-time altitude visualisation.

LiveAltitudePlotter collects telemetry (time, setpoint, altitude) during the flight
and updates the plot on the fly. The controllers call record() on every step.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class LiveAltitudePlotter:
    def __init__(self) -> None:
        import matplotlib.pyplot as plt

        self._plt = plt
        self._timestamps_s: list[float] = []
        self._setpoints_m: list[float] = []
        self._altitudes_m: list[float] = []

        # Redraw at most once per interval so drawing does not slow the control loop.
        self._redraw_interval_s = 1.0
        self._last_draw_s: float | None = None

        plt.ion()
        self._figure, self._axes = plt.subplots(figsize=(10, 5))
        (self._setpoint_line,) = self._axes.plot(
            [], [], label="Setpoint", linestyle="--",
            drawstyle="steps-post", color="tab:red",
        )
        (self._altitude_line,) = self._axes.plot(
            [], [], label="Drone altitude", color="tab:blue",
        )
        self._axes.set_xlabel("Time, s")
        self._axes.set_ylabel("Altitude, m")
        self._axes.set_title("PID altitude tracking")
        self._axes.grid(True, alpha=0.3)
        self._axes.legend()

        # Initial limits + hint so the window is not empty while there is no data yet.
        self._axes.set_xlim(0, 10)
        self._axes.set_ylim(0, 20)
        self._waiting_text = self._axes.text(
            0.5, 0.5, "Waiting for data...",
            transform=self._axes.transAxes, ha="center", va="center",
            fontsize=14, color="gray",
        )

        self._figure.show()
        self._refresh()

    def record(self, timestamp_s: float, setpoint_m: float, altitude_m: float) -> None:
        """Append a sample (cheap) and redraw the plot infrequently (expensive)."""
        self._timestamps_s.append(timestamp_s)
        self._setpoints_m.append(setpoint_m)
        self._altitudes_m.append(altitude_m)

        if self._last_draw_s is None or (timestamp_s - self._last_draw_s) >= self._redraw_interval_s:
            self._last_draw_s = timestamp_s
            self._draw()

    def _draw(self) -> None:
        if self._waiting_text is not None:
            self._waiting_text.remove()
            self._waiting_text = None

        start = self._timestamps_s[0]
        elapsed_s = [t - start for t in self._timestamps_s]
        self._setpoint_line.set_data(elapsed_s, self._setpoints_m)
        self._altitude_line.set_data(elapsed_s, self._altitudes_m)
        self._axes.relim()
        self._axes.set_autoscale_on(True)
        self._axes.autoscale_view()
        self._refresh()

    def _refresh(self) -> None:
        self._figure.canvas.draw_idle()
        self._figure.canvas.flush_events()

    def save(self, output_path: str) -> None:
        """Save the current plot to a file (e.g. PNG)."""
        self._figure.savefig(output_path, dpi=120)
        logger.info("Plot saved to %s", output_path)

    def keep_open(self) -> None:
        """Keep the window open after the mission finishes (blocks until closed)."""
        self._plt.ioff()
        self._plt.show()
