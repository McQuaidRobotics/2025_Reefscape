package igknighters.subsystems.climber.pivot;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Pivot extends Component {
  @Log protected double radians;
  @Log protected double amps;
  @Log protected double targetRads;
  @Log protected boolean controlledLastCycle;
  
  public abstract void setPositionRads(double positionRads);
  public abstract double getPositionRads();
  public abstract boolean isAtPosition(double positionRads, double toleranceRads);
  public abstract void setNeutralMode(boolean coast);
}
