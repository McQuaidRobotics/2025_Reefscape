package igknighters.subsystems.superStructure.Elevator;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Elevator extends Component {
  @Log protected double meters;
  @Log protected double volts;
  @Log protected double amps;
  @Log protected boolean isHomed;
  @Log protected boolean isLimitTrip;
  @Log protected double metersPerSecond;
  @Log protected double whatIsAtIsCheckingAgainst;
  @Log protected double whereMotorThinksItIsInMeters;
  @Log protected double whereItsTryingToGetToInMeters;

  public abstract void gotoPosition(double height);

  public abstract boolean isAtPosition(double position, double tolerance);

  public abstract void setNeutralMode(boolean shoodBeCoast);

  public abstract boolean home();
}
