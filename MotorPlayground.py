from dataclasses import dataclass
import math
from typing import overload

@dataclass
class DCMotor:
    nominal_voltage_volts: float
    stall_torque_newton_meters: float
    stall_current_amps: float
    free_current_amps: float
    free_speed_rad_per_sec: float
    resistance_ohms: float
    kv_rad_per_sec_per_volt: float
    kt_newton_meters_per_amp: float

    @staticmethod
    def make_motor(
        nominal_voltage: float,
        stall_torque_newton_meters: float,
        stall_current_amps: float,
        free_current_amps: float,
        free_speed_rad_per_sec: float,
        number_of_motors: int):
        return DCMotor(
            nominal_voltage,
            stall_torque_newton_meters * number_of_motors,
            stall_current_amps * number_of_motors,
            free_current_amps * number_of_motors,
            free_speed_rad_per_sec,
            nominal_voltage / stall_current_amps,
            free_speed_rad_per_sec / (nominal_voltage - free_current_amps * (nominal_voltage / stall_current_amps)),
            stall_torque_newton_meters / stall_current_amps
        )

    def get_current(self, speed_radians_per_sec: float, voltage_input: float) -> float:
        return (-1.0 / self.kv_rad_per_sec_per_volt / self.resistance_ohms * speed_radians_per_sec) \
            + (1.0 / self.resistance_ohms * voltage_input)

    def get_current_torque(self, torque_newton_meters: float) -> float:
        return torque_newton_meters / self.kt_newton_meters_per_amp

    def get_torque(self, current_amps: float) -> float:
        return current_amps * self.kt_newton_meters_per_amp

    def get_voltage(self, torque_newton_meters: float, speed_radians_per_sec: float) -> float:
        return (1.0 / self.kv_rad_per_sec_per_volt * speed_radians_per_sec) \
            + 1.0 / self.kt_newton_meters_per_amp * self.resistance_ohms * torque_newton_meters

    def get_speed(self, torque_newton_meters: float, voltage_input: float) -> float:
        return voltage_input * self.kv_rad_per_sec_per_volt \
            - 1.0 / self.kt_newton_meters_per_amp * torque_newton_meters * self.resistance_ohms * self.kv_rad_per_sec_per_volt

    def get_speed_percent(self, speed_rad_per_sec: float, voltage_input: float) -> float:
        return speed_rad_per_sec / (voltage_input * self.kv_rad_per_sec_per_volt)

    def get_free_current(self, speed_rad_per_sec: float, voltage_input: float) -> float:
        return self.free_current_amps * self.get_speed_percent(speed_rad_per_sec, voltage_input)

    def get_max_power(self, voltage_input: float, supply_limit_amps: float) -> float:
        return voltage_input * supply_limit_amps

    def get_max_stator(self, speed_rad_per_sec: float, voltage_input: float, supply_limit_amps: float, stator_limit_amps: float) -> float:
        v_in = abs(voltage_input)
        v_velo = abs(voltage_input * self.get_speed_percent(speed_rad_per_sec, voltage_input))
        max_p = abs(self.get_max_power(voltage_input, supply_limit_amps))
        free_current = self.get_free_current(speed_rad_per_sec, voltage_input)

        x_sqr = (v_velo * v_velo) - (4.0 * self.resistance_ohms * (-max_p + (v_in * free_current)))

        if x_sqr < 0:
            return stator_limit_amps

        y = abs(-v_velo + math.sqrt(x_sqr)) / (2.0 * self.resistance_ohms)

        return min(y, stator_limit_amps)

    def get_current_limited(self, speed_rad_per_sec: float, voltage_input: float, supply_limit_amps: float, stator_limit_amps: float) -> float:
        normal_stator = self.get_current(speed_rad_per_sec, voltage_input)
        limited_stator = self.get_max_stator(speed_rad_per_sec, voltage_input, supply_limit_amps, stator_limit_amps)
        return min(abs(normal_stator), abs(limited_stator)) * math.copysign(1, normal_stator)

    def get_output_power_torque(self, speed_rad_per_sec: float, torque_newton_meters: float) -> float:
        return speed_rad_per_sec * torque_newton_meters

    def get_output_power_current(self, speed_rad_per_sec: float, stator_current: float) -> float:
        return speed_rad_per_sec * self.get_torque(stator_current)

    def get_losses(self, speed_rad_per_sec: float, voltage_input: float, stator_current: float) -> float:
        passive_loss: float = self.get_free_current(speed_rad_per_sec, voltage_input) * voltage_input
        resistive_loss: float = (stator_current ** 2) * self.resistance_ohms
        return passive_loss + resistive_loss

    def get_efficiency(self, speed_rad_per_sec: float, voltage_input: float, stator_current: float) -> float:
        output: float = self.get_output_power_current(speed_rad_per_sec, stator_current)
        losses: float = self.get_losses(speed_rad_per_sec, voltage_input, stator_current)
        return output / (output + losses)

    def get_supply_current(self, speed_rad_per_sec: float, voltage_input: float, stator_current: float) -> float:
        if math.isclose(voltage_input, 0.0, abs_tol=0.0001):
            return 0.0
        stator_power = voltage_input * self.get_speed_percent(speed_rad_per_sec, voltage_input) * stator_current
        losses = self.get_losses(speed_rad_per_sec, voltage_input, stator_current)
        return (stator_power + losses) / voltage_input

def rpm_to_rad_per_sec(rpm: float) -> float:
    return rpm * math.pi / 30.0

KRAKEN: DCMotor = DCMotor.make_motor(12.0, 7.09, 366.0, 2.0, rpm_to_rad_per_sec(6000.0), 1)
KRAKEN_FOC: DCMotor = DCMotor.make_motor(12.0, 9.37, 483.0, 2.0, rpm_to_rad_per_sec(5800.0), 1)

speed = 0.0
voltage = 5.0
stator = KRAKEN_FOC.get_current_limited(speed, voltage, 40.0, 120.0)
# print(KRAKEN_FOC.get_voltage(KRAKEN_FOC.get_torque(stator), speed))
# print(stator)
# print(KRAKEN_FOC.get_torque(stator))
# print(KRAKEN_FOC.get_supply_current(speed, voltage, stator))



def test_cos(rad: float):
    cos = math.cos(rad)
    unary_cos = math.cos(-rad)
    if (cos == unary_cos):
        print("True")
    else:
        print("False")

def test_sin(rad: float):
    sin = math.sin(rad)
    unary_sin = math.sin(-rad)
    if (sin == unary_sin):
        print("True")
    else:
        print("False")

def test_xy(x: float, y: float):
    rad = math.atan2(y, x)
    unary_rad = math.atan2(-y, x)
    print(-rad == unary_rad)

import random
for i in range(0, 100):
    x = random.uniform(-100, 100)
    y = random.uniform(-100, 100)
    test_xy(x, y)