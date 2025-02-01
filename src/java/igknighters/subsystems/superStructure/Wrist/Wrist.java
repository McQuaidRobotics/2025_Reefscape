package igknighters.subsystems.superStructure.Wrist;

import edu.wpi.first.math.MathUtil;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Wrist extends Component {
  @Log protected double radians;
  @Log protected double targetRadians;
  @Log protected double radiansPerSecond;
  @Log protected double amps;
  @Log protected double volts;

  /**
   * Requests the wrist to go to a specific position for the next control cycle.
   *
   * @param targetPosition The target angle of the wrist in radians.
   */
  public abstract void goToPosition(double targetPosition);

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
   * @param tolerance The tolerance of the wrist in radians, use {@link
   *     WristConstants#DEFAULT_TOLERANCE} if you are unsure.
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
}
