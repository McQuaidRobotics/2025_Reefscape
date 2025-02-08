package igknighters.subsystems.intake.rollers;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Rollers extends Component {
  @Log protected double amps;
  @Log protected boolean hasAlgae;
  @Log protected boolean hasCoral;
  @Log protected double rads;

  public abstract void setVoltage(double voltage);

  public abstract boolean hasCoral();

  public abstract boolean hasAlgae();
}
