package igknighters.subsystems.superStructure;

import static igknighters.constants.ConstValues.Conv.*;
import static igknighters.constants.FieldConstants.Reef.BranchHeight.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kElevator.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kWrist.*;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.constants.ConstValues.Conv;
import monologue.ProceduralStructGenerator;

public enum SuperStructureState implements StructSerializable {
  ScoreL4(MAX_HEIGHT - (3.0 * Conv.INCHES_TO_METERS), 31.0 * DEGREES_TO_RADIANS, 1.0),
  ScoreL3(L3.height + 0.25, -L3.pitch),
  ScoreL2(L2.height + 0.25, -L2.pitch),
  ScoreL1(0.74, 29.0 * DEGREES_TO_RADIANS),
  StagedL4(L3.height + 0.30, -0.85, 2.0),
  AlgaeL3(L3.height + 0.17, 25.0 * DEGREES_TO_RADIANS),
  AlgaeL2(L2.height + 0.17, 25.0 * DEGREES_TO_RADIANS),
  AlgaeFloor(MIN_HEIGHT, 27.0 * DEGREES_TO_RADIANS),
  Stow(28.95 * INCHES_TO_METERS, MAX_ANGLE, 2.0),
  Processor(MIN_HEIGHT, 7.5 * DEGREES_TO_RADIANS, 1.5),
  Net(MAX_HEIGHT, ALGAE_MAX_ANGLE + 5.0 * Conv.DEGREES_TO_RADIANS, 1.0),
  IntakeHpClose(28.95 * INCHES_TO_METERS, -54.0 * DEGREES_TO_RADIANS),
  IntakeHpFar(26.2 * INCHES_TO_METERS, -48.0 * DEGREES_TO_RADIANS),
  AntiTilt(MIN_HEIGHT + 0.05, MAX_ANGLE, 1.0);

  
  public final double elevatorMeters;
  public final double wristRads;
  public final double toleranceScalar;

  SuperStructureState(double elevatorMeters, double wristRads, double toleranceScalar) {
    this.elevatorMeters = elevatorMeters;
    this.wristRads = wristRads;
    this.toleranceScalar = toleranceScalar;
  }

  SuperStructureState(double elevatorMeters, double wristRads) {
    this.elevatorMeters = elevatorMeters;
    this.wristRads = wristRads;
    this.toleranceScalar = 1.0;
  }

  public SuperStructureState minHeight(SuperStructureState other) {
    return this.elevatorMeters < other.elevatorMeters ? this : other;
  }

  public SuperStructureState maxHeight(SuperStructureState other) {
    return this.elevatorMeters > other.elevatorMeters ? this : other;
  }

  public static final Struct<SuperStructureState> struct =
      ProceduralStructGenerator.genEnum(SuperStructureState.class);
}
