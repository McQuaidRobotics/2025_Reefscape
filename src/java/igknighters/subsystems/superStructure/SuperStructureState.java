package igknighters.subsystems.superStructure;

import igknighters.constants.ConstValues.Conv;
import igknighters.constants.FieldConstants.ReefHeight;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants;
import igknighters.subsystems.superStructure.Wrist.WristConstants;

public enum SuperStructureState {
  ScoreL4(ReefHeight.L4.height, ReefHeight.L4.pitch, 1.0),
  ScoreL3(ReefHeight.L3.height, ReefHeight.L3.pitch, 1.0),
  ScoreL2(ReefHeight.L2.height, ReefHeight.L2.pitch, 1.0),
  ScoreL1(ReefHeight.L1.height, ReefHeight.L1.pitch, 1.0),
  AlgaeL3(51.0 * Conv.INCHES_TO_METERS, 20.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  AlgaeL2(37.0 * Conv.INCHES_TO_METERS, 20.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  Stow(ElevatorConstants.MIN_HEIGHT + 0.3, WristConstants.MAX_ANGLE * 0.98, 1.0),
  Processor(20.0 * Conv.INCHES_TO_METERS, 0.0, 1.0),
  Net(ElevatorConstants.MAX_HEIGHT * 0.98, 60.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  IntakeHp(32.0 * Conv.INCHES_TO_METERS, 35.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  AlgaeIntakeGround(0.0, -35.0 * Conv.DEGREES_TO_RADIANS, 1.0);

  public final double elevatorMeters;
  public final double wristRads;
  public final double toleranceScalar;

  SuperStructureState(double elevatorMeters, double wristRads, double toleranceScalar) {
    this.elevatorMeters = elevatorMeters;
    this.wristRads = wristRads;
    this.toleranceScalar = toleranceScalar;
  }
}
