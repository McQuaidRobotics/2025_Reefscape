package igknighters.subsystems.intake.rollers;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Rollers extends Component {

  @Log protected double amps;
  @Log protected double volts;
  @Log protected double radiansPerSecond;
  @Log protected boolean laserTripped;
  @Log protected double gamepieceDistance;
  @Log protected boolean controlledLastCycle;

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

  public boolean isLaserTripped() {
    return laserTripped;
  }

  public abstract boolean isStalling();

  public double gamepieceDistance() {
    return gamepieceDistance;
  }
}
