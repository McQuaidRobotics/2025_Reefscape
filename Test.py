from wayfinder.Plotters import plot_trajectory, Trajectory

from wayfinder.DynamicTrapezoidalProfile import calculate as calculate_trapezoidal
from wayfinder.DynamicTrapezoidalProfile import Constraints as DynConstraints, State as DynState

STARTING = DynState(1.0, 0.0)
GOAL = DynState(5.0, 0.0)
CONSTRAINTS = DynConstraints(1.0, 3.0)

trajectory = Trajectory([], [], [])
current = STARTING
for i in range(100000):
    time = i * 0.01
    current = calculate_trapezoidal(0.01, current, GOAL, CONSTRAINTS)
    trajectory.add_sample(time, current.position, current.velocity)

    if (current.epsilon_equals(GOAL)):
        break

plot_trajectory(trajectory)


from wayfinder.TrapezoidalProfile import TrapezoidProfile, State, Constraints

PROFILE = TrapezoidProfile(Constraints(1.0, 3.0))
STARTING = State(1.0, 0.0)
GOAL = State(5.0, 0.0)

trajectory = Trajectory([], [], [])
current = STARTING
for i in range(100000):
    time = i * 0.01
    current = PROFILE.calculate(0.01, current, GOAL)
    trajectory.add_sample(time, current.position, current.velocity)

    if (abs(current.position - GOAL.position) < 0.01 and abs(current.velocity - GOAL.velocity) < 0.01):
        break

plot_trajectory(trajectory)