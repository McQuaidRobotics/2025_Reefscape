from dataclasses import dataclass

@dataclass
class State:
    """State of the trapezoidal profile."""
    position: float = 0.0
    velocity: float = 0.0

    def __str__(self) -> str:
        return f"Position: {self.position}, Velocity: {self.velocity}"

    def epsilon_equals(self, other: 'State', epsilon: float = 1e-6) -> bool:
        """Check if two states are approximately equal."""
        return abs(self.position - other.position) < epsilon and abs(self.velocity - other.velocity) < epsilon

@dataclass
class Constraints:
    """Constraints for the trapezoidal profile."""
    max_velocity: float = 0.0
    max_acceleration: float = 0.0

    def __str__(self) -> str:
        return f"Max Velocity: {self.max_velocity}, Max Acceleration: {self.max_acceleration}"

def __should_flip_acceleration(initial_position: float, goal_position: float) -> bool:
    """Determine if the acceleration should be flipped based on the initial and goal positions."""
    return initial_position > goal_position

def __direct(number: float, forward: bool) -> float:
    """Direct the number based on the forward direction."""
    direction = -1.0 if forward else 1.0
    return number * direction

def calculate(
    period: float,
    current: State,
    goal: State,
    constraints: Constraints
) -> State:
    """Calculate the trapezoidal profile state."""
    direction = __should_flip_acceleration(current.position, goal.position)
    current_pos = __direct(current.position, direction)
    current_vel = __direct(current.velocity, direction)
    goal_pos = __direct(goal.position, direction)
    goal_vel = __direct(goal.velocity, direction)
    max_vel = constraints.max_velocity
    max_accel = constraints.max_acceleration

    if current_vel > max_vel:
        current_vel = max_vel

    # Deal with a possibly truncated motion profile (with nonzero initial or final velocity) by calculating the parameters as if the profile began and ended at zero velocity
    cutoff_begin = current_vel / max_accel
    cutoff_dist_begin = cutoff_begin * cutoff_begin * max_accel / 2.0

    cutoff_end = goal_vel / max_accel
    cutoff_dist_end = cutoff_end * cutoff_end * max_accel / 2.0

    # Now we can calculate the parameters as if it was a full trapezoid instead of a truncated one
    full_trapezoid_dist = cutoff_dist_begin + (goal_pos - current_pos) + cutoff_dist_end
    acceleration_time = max_vel / max_accel

    full_speed_dist = full_trapezoid_dist - acceleration_time * acceleration_time * max_accel

    # Handle the case where the profile never reaches full speed
    if full_speed_dist < 0:
        acceleration_time = (full_trapezoid_dist / max_accel) ** 0.5
        full_speed_dist = 0

    end_accel = acceleration_time - cutoff_begin
    end_full_speed = end_accel + full_speed_dist / max_vel
    end_decel = end_full_speed + acceleration_time - cutoff_end

    result_pos = current_pos
    result_vel = current_vel

    if period < end_accel:
        result_vel += period * max_accel
        result_pos += (current_vel + period * max_accel / 2.0) * period
    elif period < end_full_speed:
        result_vel = max_vel
        result_pos += (current_vel + end_accel * max_accel / 2.0) * end_accel + max_vel * (period - end_accel)
    elif period <= end_decel:
        result_vel = goal_vel + (end_decel - period) * max_accel
        time_left = end_decel - period
        result_pos = goal_pos - (goal_vel + time_left * max_accel / 2.0) * time_left
    else:
        result_pos = goal_pos
        result_vel = goal_vel

    return State(__direct(result_pos, direction), __direct(result_vel, direction))