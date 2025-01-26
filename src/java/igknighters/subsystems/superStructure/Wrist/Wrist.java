package igknighters.subsystems.superStructure.Wrist;

import edu.wpi.first.math.MathUtil;
import igknighters.subsystems.Component;

public abstract class Wrist extends Component {

  public abstract void goToPosition(double angleDegrees);

  public boolean isAtPosition(double angleDegrees, double toleranceDegrees) {
    return MathUtil.isNear(angleDegrees, this.positionRadians(), toleranceDegrees);
  }

  public abstract void setNeutralMode(boolean shouldBeCoast);

  public abstract double positionRadians();
}
