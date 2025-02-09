package igknighters.subsystems.intake.rollers;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Rollers extends Component {
  @Log protected double current;
  @Log protected double volts;
  @Log protected boolean hasAlgae;
  @Log protected boolean hasCoral;
  @Log protected double radiansPerSecond;

  public abstract void setVoltage(double voltage);

  public abstract void setTorque(double torque);

  public abstract boolean hasCoral();

  public abstract boolean hasAlgae();
}
