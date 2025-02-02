package igknighters.subsystems.superStructure.Elevator;

import edu.wpi.first.math.MathUtil;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Elevator extends Component {
  @Log protected double meters;
  @Log protected double targetMeters;
  @Log protected double metersPerSecond;
  @Log protected double volts;
  @Log protected double amps;
  @Log protected boolean isHomed;
  @Log protected boolean isLimitTripped;
  @Log protected boolean controlRequestedLastCycle;

  /**
   * Requests the elevator to go to a specific position for the next control cycle.
   *
   * @param targetPosition The target position of the elevator in meters.
   */
  public abstract void gotoPosition(double targetPosition);

  /**
   * Returns the current position of the elevator in meters.
   *
   * @return The position of the elevator in meters.
   */
  public double position() {
    return meters;
  }

  /**
   * Returns the current position of the elevator
   *
   * @param position The position of the elevator in meters.
   * @param tolerance The tolerance of the elevator in meters, use {@link
   *     ElevatorConstants#DEFAULT_TOLERANCE} if you are unsure.
   * @return True if the elevator is at the desired position, false otherwise.
   */
  public boolean isAtPosition(double position, double tolerance) {
    return MathUtil.isNear(position, meters, tolerance);
  }

  /**
   * Sets the neutral mode of the elevator motor controller.
   *
   * @param coast True to set the motor controller to coast mode, false to set it to brake mode.
   */
  public abstract void setNeutralMode(boolean coast);

  /**
   * Homes the elevator.
   *
   * @return True if the elevator is homed, false otherwise.
   */
  public boolean home() {
    if (isLimitTripped || isHomed) {
      isHomed = true;
    } else {
      voltageOut(ElevatorConstants.HOMING_VOLTAGE);
    }
    return isHomed;
  }

  /**
   * Requests the elevator to be powered at a specific voltage for the next control cycle.
   *
   * @param voltage The voltage to power the elevator motor controller.
   */
  public abstract void voltageOut(double voltage);
}
