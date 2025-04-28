import matplotlib.pyplot as plt
from dataclasses import dataclass
from wpimath.geometry import Translation2d

@dataclass
class Trajectory2d:
    """Trajectory of the theoretical chassis."""
    time: list[float]
    position: list[Translation2d]
    velocity: list[Translation2d]

def plot_trajectory2d(trajectory: Trajectory2d) -> None:
    """Plot the trajectory of the theoretical chassis."""
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))

    # Plot position
    ax[0].scatter([p.x for p in trajectory.position], [p.y for p in trajectory.position], label='Trajectory')
    ax[0].set_title('Trajectory')
    ax[0].set_xlabel('X Position (m)')
    ax[0].set_ylabel('Y Position (m)')
    ax[0].grid()
    ax[0].legend()
    ax[0].set_xlim(0, 10)
    ax[0].set_ylim(0, 10)

    # Plot velocity
    ax[1].plot(trajectory.time, [v.x for v in trajectory.velocity], label='X Velocity', color='r')
    ax[1].plot(trajectory.time, [v.y for v in trajectory.velocity], label='Y Velocity', color='g')
    ax[1].set_title('Velocity')
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Velocity (m/s)')
    ax[1].grid()
    ax[1].legend()

    plt.tight_layout()
    plt.show()

@dataclass
class Trajectory:
    """Trajectory of the theoretical chassis."""
    time: list[float]
    position: list[float]
    velocity: list[float]

    def add_sample(self, time: float, position: float, velocity: float) -> None:
        """Add a sample to the trajectory."""
        self.time.append(time)
        self.position.append(position)
        self.velocity.append(velocity)

def plot_trajectory(trajectory: Trajectory) -> None:
    """Plot the trajectory of the theoretical chassis."""
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))

    # Plot position
    ax[0].plot(trajectory.time, trajectory.position, label='Trajectory')
    ax[0].set_title('Trajectory')
    ax[0].set_xlabel('Time (s)')
    ax[0].set_ylabel('Position (m)')
    ax[0].grid()
    ax[0].legend()

    # Plot velocity
    ax[1].plot(trajectory.time, trajectory.velocity, label='Velocity', color='r')
    ax[1].set_title('Velocity')
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Velocity (m/s)')
    ax[1].grid()
    ax[1].legend()

    plt.tight_layout()
    plt.show()