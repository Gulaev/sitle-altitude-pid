"""
Minimal PID controller with output clamping and anti-windup.

    output = Kp*e + Ki*integral(e*dt) + Kd*(de/dt),  e = setpoint - measurement
"""


class PIDController:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        """Reset internal state. Call when the target changes (a new flight phase)."""
        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, setpoint, measurement, dt):
        """Run one control step and return the control signal, clamped to the output limits."""
        error = setpoint - measurement
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        self.integral += error * dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Anti-windup (conditional integration): while the output is saturated, roll
        # back this step's accumulation so the integral does not wind up and overshoot.
        if self.output_max is not None and output > self.output_max:
            self.integral -= error * dt
            output = self.output_max
        elif self.output_min is not None and output < self.output_min:
            self.integral -= error * dt
            output = self.output_min

        return output
