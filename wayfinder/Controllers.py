
from dataclasses import dataclass

from wayfinder.DynamicTrapezoidalProfile import State, Constraints, calculate
from wpimath.geometry import Translation2d, Rotation2d

def speed_in_direction(translation: Translation2d, direction: Rotation2d) -> float:
    if abs(translation.norm()) < 1e-6:
        return 0.0
    return translation.x * direction.cos() + translation.y * direction.sin()

class ProfiledTranslationController:

    def __init__(self, k_P: float, k_I: float, k_D: float):
        self.k_P = k_P
        self.k_I = k_I
        self.k_D = k_D
        self.prev_error = 0.0
        self.total_error = 0.0
        self.prev_setpoint = State.zero()

    def is_done(self, measurement: Translation2d, target: Translation2d) -> bool:
        return self.prev_setpoint.epsilon_equals(State.zero())

    def reset(self, measurement: Translation2d, measurement_velocity: Translation2d, target: Translation2d):
        self.prev_error = 0.0
        self.total_error = 0.0
        direction: Rotation2d = (target - measurement).angle()
        distance = (target - measurement).norm()
        self.prev_setpoint = State(-distance, speed_in_direction(measurement_velocity, direction))

    def calculate(
        self,
        period: float,
        measurement: Translation2d,
        measurement_velocity: Translation2d,
        target: Translation2d,
        constraints: Constraints
    ) -> Translation2d:
        direction: Rotation2d = (target - measurement).angle()
        distance = (target - measurement).norm()

        setpoint: State = calculate(
            period,
            self.prev_setpoint,
            State.zero(),
            constraints
        )

        print(f"setpoint: {setpoint}")

        position_error = distance + setpoint.position

        error_derivative = (position_error - self.prev_error) / period
        if (self.k_I > 0.0):
            self.total_error += position_error * period

        self.prev_error = position_error
        self.prev_setpoint = setpoint

        dir_velo = setpoint.velocity + self.k_P * position_error + self.k_I * self.total_error + self.k_D * error_derivative

        return Translation2d(
            dir_velo * direction.cos(),
            dir_velo * direction.sin()
        )