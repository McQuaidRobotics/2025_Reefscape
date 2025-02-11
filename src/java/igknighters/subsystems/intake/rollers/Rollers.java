package igknighters.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Rollers extends Component {
  private final double kt, kv;

  @Log protected double current;
  @Log protected double volts;
  @Log protected boolean hasAlgae;
  @Log protected boolean hasCoral;
  @Log protected double radiansPerSecond;

  protected Rollers(DCMotor motor) {
    kt = motor.KtNMPerAmp;
    kv = motor.KvRadPerSecPerVolt;
  }

  /**
   * Sets the voltage of the rollers.
   *
   * @param voltage the voltage to set
   */
  public abstract void setVoltage(double voltage);

  /**
   * Sets the current of the rollers.
   *
   * @param current the current to set
   */
  public abstract void setCurrent(double current);

  /**
   * Sets the torque of the rollers.
   *
   * @param torque the torque to set in newton-meters
   */
  public void setTorque(double torque) {
    setCurrent(torque / kt);
  }

  /**
   * Sets the velocity of the rollers.
   *
   * @param velocity the velocity to set in radians per second
   */
  public void setVelocity(double velocity) {
    setVoltage(velocity / kv);
  }

  /**
   * Returns true if the rollers have coral.
   *
   * <p>This returning true in a given cycle should be mutually exclusive with {@link #hasAlgae()}
   * returning true in the same cycle.
   *
   * @return true if the rollers have coral
   */
  public abstract boolean hasCoral();

  /**
   * Returns true if the rollers have algae.
   *
   * <p>This returning true in a given cycle should be mutually exclusive with {@link #hasCoral()}
   * returning true in the same cycle.
   *
   * @return true if the rollers have algae
   */
  public abstract boolean hasAlgae();
}
