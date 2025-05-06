import random
from pyparsing import C
from wayfinder.Plotters import plot_trajectory, Trajectory, Trajectory2d, plot_trajectory2d

from wayfinder.DynamicTrapezoidalProfile import calculate as calculate_trapezoidal
from wayfinder.DynamicTrapezoidalProfile import Constraints as DynConstraints, State as DynState
from wayfinder.Controllers import *

# STARTING = DynState(1.0, 0.0)
# GOAL = DynState(5.0, 0.0)
# CONSTRAINTS = DynConstraints(1.0, 3.0)

# trajectory = Trajectory([], [], [])
# current = STARTING
# for i in range(100000):
#     time = i * 0.01
#     current = calculate_trapezoidal(0.01, current, GOAL, CONSTRAINTS)
#     trajectory.add_sample(time, current.position, current.velocity)

#     if (current.epsilon_equals(GOAL)):
#         break

# plot_trajectory(trajectory)

def noise(value: float, noise: float) -> float:
    return value + (random.random() - 0.5) * noise

PERIOD = 0.01
STARTING = Translation2d(1.0, 1.0)
GOAL = Translation2d(4.0, 8.0)
CONSTRAINTS = Constraints(1.0, 3.0)
CONTROLLER = ProfiledTranslationController(10.0, 0.0, 0.0)

trajectory = Trajectory2d([], [], [])

current = STARTING
velocity = Translation2d(0.0, 0.0)
CONTROLLER.reset(
    current,
    velocity,
    GOAL
)
for i in range(20000):
    print(f"i: {i}, current: {current}, velocity: {velocity}")
    velocity = CONTROLLER.calculate(
        PERIOD,
        current,
        velocity,
        GOAL,
        CONSTRAINTS
    )

    current += velocity * PERIOD * noise(0.87, 0.12)
    trajectory.add_sample(i * PERIOD, current, velocity)

    if (CONTROLLER.is_done(current, GOAL)):
        break


print(f"error: {(current - GOAL).norm()}")
# plot_trajectory2d(trajectory)