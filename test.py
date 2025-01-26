import math
from typing import Callable

def unwrap_angle(ref: float, angle: float) -> float:
    diff = angle - ref
    if diff > math.pi:
        return angle - 2.0 * math.pi
    elif diff < -math.pi:
        return angle + 2.0 * math.pi
    else:
        return angle

def input_modulus(input: float, minimum_input: float, maximum_input: float) -> float:
    modulus = maximum_input - minimum_input

    # Wrap input if it's above the maximum input
    num_max = int((input - minimum_input) / modulus)
    input -= num_max * modulus

    # Wrap input if it's below the minimum input
    num_min = int((input - maximum_input) / modulus)
    input -= num_min * modulus

    return input

def angle_modulus(angle_radians: float) -> float:
    return input_modulus(angle_radians, -math.pi, math.pi)


def find_root(
        func: Callable[[float, float], float],
        x_0: float,
        y_0: float,
        f_0: float,
        x_1: float,
        y_1: float,
        f_1: float,
        iterations_left: int
  ) -> float:
    s_guess = max(0.0, min(1.0, -f_0 / (f_1 - f_0)))

    if iterations_left < 0 or abs(f_0 - f_1) < 1e-9:
        return s_guess

    x_guess = (x_1 - x_0) * s_guess + x_0
    y_guess = (y_1 - y_0) * s_guess + y_0
    f_guess = func(x_guess, y_guess)
    if math.copysign(1, f_0) == math.copysign(1, f_guess):
        # 0 and guess on same side of root, so use upper bracket.
        return s_guess + (1.0 - s_guess) * find_root(
            func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1)
    else:
        # Use lower bracket.
        return s_guess * find_root(
            func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1)
