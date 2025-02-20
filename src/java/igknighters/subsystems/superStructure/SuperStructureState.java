package igknighters.subsystems.superStructure;

import static igknighters.constants.ConstValues.Conv.*;
import static igknighters.constants.FieldConstants.Reef.BranchHeight.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.ElevatorConstants.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.WristConstants.*;

import igknighters.subsystems.superStructure.SuperStructureConstants.WristConstants;

public enum SuperStructureState {
  ScoreL4(MAX_HEIGHT, -10.0 * DEGREES_TO_RADIANS, 0.75),
  ScoreL3(L3.height + wristYOffset(L3.pitch), -L3.pitch, 1.0),
  ScoreL2(L2.height + wristYOffset(L2.pitch), -L2.pitch, 1.0),
  ScoreL1(L1.height, 10.0 * DEGREES_TO_RADIANS, 1.5),
  AlgaeL3(L3.height + 0.1, -10.0 * DEGREES_TO_RADIANS, 1.2),
  AlgaeL2(L2.height + 0.1, -10.0 * DEGREES_TO_RADIANS, 1.2),
  Stow(MIN_HEIGHT + STAGES[3].rangeOfMotion(), MIN_ANGLE, 2.0),
  Processor(20.0 * INCHES_TO_METERS, 0.0, 1.5),
  Net(MAX_HEIGHT, MAX_ANGLE, 1.0),
  IntakeHp(32.0 * INCHES_TO_METERS, 55.0 * DEGREES_TO_RADIANS, 0.8);

  public final double elevatorMeters;
  public final double wristRads;
  public final double toleranceScalar;

  private static double wristYOffset(double theta) {
    return WristConstants.LENGTH * Math.sin(theta);
  }

  SuperStructureState(double elevatorMeters, double wristRads, double toleranceScalar) {
    this.elevatorMeters = elevatorMeters;
    this.wristRads = wristRads;
    this.toleranceScalar = toleranceScalar;
  }
}
