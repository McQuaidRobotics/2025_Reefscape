package igknighters.subsystems.superStructure.Wrist;

import edu.wpi.first.math.MathUtil;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Wrist extends Component {
  @Log protected double angleRadians;
  @Log protected double amps;
  @Log protected boolean isHomed;
  @Log protected double radiansPerSecond;
  @Log protected double targetRadians;

  public abstract void goToPosition(double angleRadians);

  public boolean isAtPosition(double angleRadians, double toleranceDegrees) {
    return MathUtil.isNear(angleRadians, this.positionRadians(), toleranceDegrees);
  }

  public abstract void setNeutralMode(boolean shouldBeCoast);

  public abstract double positionRadians();
}
