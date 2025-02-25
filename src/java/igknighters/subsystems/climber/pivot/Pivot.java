package igknighters.subsystems.climber.pivot;

import edu.wpi.first.math.MathUtil;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Pivot extends Component {
  @Log protected double targetRads;
  @Log protected double radians;
  @Log protected double amps;
  @Log protected double volts;
  @Log protected boolean controlledLastCycle;

  public abstract void setPositionRads(double positionRads);

  public double getPositionRads() {
    return radians;
  }

  public boolean isAtPosition(double positionRads, double toleranceRads) {
    return MathUtil.isNear(positionRads, getPositionRads(), toleranceRads);
  }

  public abstract void setNeutralMode(boolean coast);

  public abstract void voltageOut(double voltage);
}
