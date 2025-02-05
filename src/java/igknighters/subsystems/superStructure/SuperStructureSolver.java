package igknighters.subsystems.superStructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.subsystems.superStructure.Wrist.WristConstants;

public class SuperStructureSolver {
  private final double wristLength = WristConstants.LENGTH;
  private final Rectangle2d boundingBox =
      new Rectangle2d(new Translation2d(0.0, 10.0), new Translation2d(24.0, 0.0));
  /**
   * Checks for potential colisions
   *
   * @param elevHeight The height of the elevator in meters
   * @param theta
   * @return position the wrist should be at in rads
   */
  public double solve(double elevHeight, double theta) {
    final double yPosition = elevHeight - boundingBox.getMeasureYWidth().in(Meters);
    if (wristLength * wristLength - yPosition * yPosition > 0.0) {
      final double xPosition = Math.sqrt(wristLength * wristLength - yPosition * yPosition);
      final double desiredThetaInRads =
          Math.acos(
              (xPosition * xPosition - yPosition * yPosition - wristLength * wristLength)
                  / (-2.0 * yPosition * wristLength));
      return desiredThetaInRads;

    } else if (wristLength - yPosition < 0.0) {
      return Math.PI / 6.0;

    } else {
      return theta;
    }
  }
}
