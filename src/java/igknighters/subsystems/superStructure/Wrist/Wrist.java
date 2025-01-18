package igknighters.subsystems.superStructure.Wrist;

import igknighters.subsystems.Component;

public abstract class Wrist extends Component {

  public abstract void goToPosition(double angleDegrees);

  public abstract boolean isAtPosition(double angleDegrees, double toleranceDegrees);

  public abstract void setNeutralMode(boolean shouldBeCoast);
  
}
