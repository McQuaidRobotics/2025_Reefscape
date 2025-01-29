package igknighters.subsystems.superStructure.Wrist;

import edu.wpi.first.math.MathUtil;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Wrist extends Component {
  @Log protected double angleDegrees;
  @Log protected double amps;
  @Log protected boolean isHomed;
  @Log protected double radiansPerSecond;
  @Log protected double targetingDegrees;

  public abstract void goToPosition(double angleDegrees);

  public boolean isAtPosition(double angleDegrees, double toleranceDegrees) {
    return MathUtil.isNear(angleDegrees, this.positionRadians(), toleranceDegrees);
  }

  public abstract void setNeutralMode(boolean shouldBeCoast);

  public abstract double positionRadians();
}
