package igknighters.subsystems.superStructure.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import igknighters.subsystems.Component;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import java.util.Optional;
import monologue.Annotations.Log;

public abstract class Wrist extends Component {
  protected static final Constraints DEFAULT_CONSTRAINTS =
      new Constraints(kWrist.MAX_VELOCITY, kWrist.MAX_ACCELERATION);

  @Log protected double radians;
  @Log protected double targetRadians;
  @Log protected double radiansPerSecond;
  @Log protected double amps;
  @Log protected double volts;
  @Log protected boolean controlledLastCycle;
  @Log protected double maxVelocity;
  @Log protected double maxAcceleration;

  /**
   * Requests the wrist to go to a specific position for the next control cycle.
   *
   * @param targetPosition The target angle of the wrist in radians.
   * @param constraints The constraints to apply to the wrist while moving.
   */
  public abstract void goToPosition(double targetPosition, Optional<Constraints> constraints);

  /**
   * Requests the wrist to go to a specific position for the next control cycle.
   *
   * @param targetPosition The target angle of the wrist in radians.
   */
  public void goToPosition(double targetPosition) {
    goToPosition(targetPosition, Optional.empty());
  }

  /**
   * Returns the current angle of the wrist in radians.
   *
   * @return The angle of the wrist in radians.
   */
  public double position() {
    return radians;
  }

  /**
   * Returns the current angle of the wrist
   *
   * @param position The angle of the wrist in radians.
   * @param tolerance The tolerance of the wrist in radians, use {@link kWrist#DEFAULT_TOLERANCE} if
   *     you are unsure.
   * @return True if the wrist is at the desired position, false otherwise.
   */
  public boolean isAtPosition(double position, double tolerance) {
    return MathUtil.isNear(position, radians, tolerance);
  }

  /**
   * Sets the neutral mode of the wrist motor controller.
   *
   * @param coast True to set the motor controller to coast mode, false to set it to brake mode.
   */
  public abstract void setNeutralMode(boolean coast);

  /**
   * Requests the wrist to be powered at a specific voltage for the next control cycle.
   *
   * @param voltage The voltage to power the wrist motor controller.
   */
  public abstract void voltageOut(double voltage);

  protected final void noTarget() {
    targetRadians = Double.NaN;
    maxVelocity = Double.NaN;
    maxAcceleration = Double.NaN;
  }
}
