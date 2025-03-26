package igknighters.subsystems.superStructure;

import static igknighters.constants.ConstValues.Conv.*;
import static igknighters.constants.FieldConstants.Reef.BranchHeight.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kElevator.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kWrist.*;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.ProceduralStructGenerator;

public enum SuperStructureState implements StructSerializable {
  ScoreL4(MAX_HEIGHT, 29.0 * DEGREES_TO_RADIANS, 1.0),
  ScoreL3(L3.height + 0.25, -L3.pitch),
  ScoreL2(L2.height + 0.25, -L2.pitch),
  ScoreL1(0.74, 29.0 * DEGREES_TO_RADIANS),
  StagedL4(L3.height + 0.20, MAX_ANGLE, 2.0),
  StagedL3(L3.height + 0.25, MAX_ANGLE, 2.0),
  StagedL2(L2.height + 0.25, MAX_ANGLE, 2.0),
  AlgaeL3(L3.height + 0.17, 25.0 * DEGREES_TO_RADIANS),
  AlgaeL2(L2.height + 0.17, 25.0 * DEGREES_TO_RADIANS),
  AlgaeFloor(MIN_HEIGHT, 15.0 * DEGREES_TO_RADIANS),
  Stow(MIN_HEIGHT + STAGES[2].rangeOfMotion() - 3.0 * INCHES_TO_METERS, MAX_ANGLE, 2.0),
  Processor(MIN_HEIGHT, 0.0, 1.5),
  Net(MAX_HEIGHT - 0.15, MIN_ANGLE, 1.0),
  Net_FLICKED(MAX_HEIGHT, MAX_ANGLE, 1.0),
  // Net(L3.height, -1.0, 1.0),
  // Net_FLICKED(MAX_HEIGHT, -1.0, 1.0),
  IntakeHpClose(28.25 * INCHES_TO_METERS, -59.0 * DEGREES_TO_RADIANS),
  IntakeHpFar(26.2 * INCHES_TO_METERS, -53.0 * DEGREES_TO_RADIANS),
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
