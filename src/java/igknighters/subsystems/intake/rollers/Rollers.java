package igknighters.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Rollers extends Component {
  private final double kt, kv;

  @Log protected double amps;
  @Log protected double volts;
  @Log protected boolean laserTripped;
  @Log protected double radiansPerSecond;
  @Log protected boolean controlledLastCycle;

  protected Rollers(DCMotor motor) {
    kt = motor.KtNMPerAmp;
    kv = motor.KvRadPerSecPerVolt;
  }

  /**
   * Sets the voltage of the rollers.
   *
   * @param voltage the voltage to set
   */
  public abstract void voltageOut(double voltage);

  /**
   * Sets the current of the rollers.
   *
   * @param current the current to set
   */
  public abstract void currentOut(double current);

  /**
   * Sets the torque of the rollers.
   *
   * @param torque the torque to set in newton-meters
   */
  public void torqueOut(double torque) {
    currentOut(torque / kt);
  }

  /**
   * Sets the velocity of the rollers.
   *
   * @param velocity the velocity to set in radians per second
   */
  public void velocityOut(double velocity) {
    voltageOut(velocity / kv);
  }

  public boolean isLaserTripped() {
    return false;
  }

  public double currentDraw() {
    return amps;
  }
}
