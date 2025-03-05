package igknighters.subsystems;

import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import wpilibExt.Speeds.FieldSpeeds;

public class SharedState {
  public double elevatorHeight = kElevator.MIN_HEIGHT;
  public boolean holdingAlgae = false;
  public FieldSpeeds fieldSpeeds = FieldSpeeds.kZero;

  public static double calcCgHeight(double elevatorHeight) {
    double t =
        (elevatorHeight - kElevator.MIN_HEIGHT) / (kElevator.MAX_HEIGHT - kElevator.MIN_HEIGHT);
    return kRobotIntrinsics.MIN_CG + t * (kRobotIntrinsics.MAX_CG - kRobotIntrinsics.MIN_CG);
  }

  public static double maximumAcceleration(double elevatorHeight) {
    return ((9.81 * (kSwerve.DRIVEBASE_WIDTH / 2.0)) / calcCgHeight(elevatorHeight)) * 0.8;
  }

  public double maximumAcceleration() {
    return maximumAcceleration(elevatorHeight);
  }
}
