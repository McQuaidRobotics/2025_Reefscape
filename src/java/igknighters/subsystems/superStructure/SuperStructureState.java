package igknighters.subsystems.superStructure;

import static igknighters.constants.ConstValues.Conv.*;
import static igknighters.constants.FieldConstants.Reef.BranchHeight.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kElevator.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kWrist.*;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.ProceduralStructGenerator;

public enum SuperStructureState implements StructSerializable {
  ScoreL4(MAX_HEIGHT, 31.0 * DEGREES_TO_RADIANS, 0.75),
  ScoreL3(L3.height + 0.25, -L3.pitch),
  ScoreL2(L2.height + 0.25, -L2.pitch),
  ScoreL1(0.74, 31.0 * DEGREES_TO_RADIANS),
  AlgaeL3(L3.height + 0.23, 30.0 * DEGREES_TO_RADIANS),
  AlgaeL2(L2.height + 0.23, 30.0 * DEGREES_TO_RADIANS),
  AlgaeFloor(MIN_HEIGHT, 15.0 * DEGREES_TO_RADIANS),
  Stow(MIN_HEIGHT + STAGES[2].rangeOfMotion() - 3.0 * INCHES_TO_METERS, MAX_ANGLE, 2.0),
  ScoreStaged(L3.height + 0.29, -45.0 * DEGREES_TO_RADIANS, 2.0),
  Processor(15.0 * INCHES_TO_METERS, 0.0, 1.5),
  Net(MAX_HEIGHT, MAX_ANGLE, 1.0),
  IntakeHpClose(26.85 * INCHES_TO_METERS, -69.0 * DEGREES_TO_RADIANS),
  IntakeHpFar(26.4 * INCHES_TO_METERS, -56.0 * DEGREES_TO_RADIANS),
  AntiTilt(MIN_HEIGHT, MAX_ANGLE, 1.0);

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
