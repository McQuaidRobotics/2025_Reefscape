package igknighters.subsystems;

import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import wpilibExt.Speeds.FieldSpeeds;

public class SharedState {
  public double elevatorHeight = kElevator.MIN_HEIGHT;
  public boolean holdingAlgae = false;
  public FieldSpeeds fieldSpeeds = FieldSpeeds.kZero;

  public double calcCgHeight() {
    double t =
        (elevatorHeight - kElevator.MIN_HEIGHT) / (kElevator.MAX_HEIGHT - kElevator.MIN_HEIGHT);
    return kRobotIntrinsics.MIN_CG + t * (kRobotIntrinsics.MAX_CG - kRobotIntrinsics.MIN_CG);
  }

  public double maximumAcceleration() {
    return ((9.81 * (kSwerve.DRIVEBASE_WIDTH / 2.0)) / calcCgHeight()) * 0.8;
  }
}
