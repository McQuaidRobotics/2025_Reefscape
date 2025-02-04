package igknighters.subsystems.superStructure;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.FieldConstants.Reef.BranchHeight;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants;
import igknighters.subsystems.superStructure.Wrist.WristConstants;
import monologue.ProceduralStructGenerator;

public enum SuperStructureState implements StructSerializable {
  ScoreL4(BranchHeight.L4.height, 20.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  ScoreL3(BranchHeight.L3.height, 50.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  ScoreL2(BranchHeight.L2.height, -20.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  ScoreL1(BranchHeight.L1.height, -50.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  AlgaeL3(51.0 * Conv.INCHES_TO_METERS, 20.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  AlgaeL2(37.0 * Conv.INCHES_TO_METERS, 20.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  Stow(ElevatorConstants.REVERSE_LIMIT + 0.3, WristConstants.FORWARD_LIMIT * 0.98, 1.0),
  Processor(20.0 * Conv.INCHES_TO_METERS, 0.0, 1.0),
  Net(ElevatorConstants.FORWARD_LIMIT * 0.98, 60.0 * Conv.DEGREES_TO_RADIANS, 1.0),
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

  public static final Struct<SuperStructureState> struct =
      ProceduralStructGenerator.genEnum(SuperStructureState.class);
}
