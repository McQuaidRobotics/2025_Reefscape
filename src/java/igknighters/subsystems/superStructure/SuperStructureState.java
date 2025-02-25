package igknighters.subsystems.superStructure;

import static igknighters.constants.ConstValues.Conv.*;
import static igknighters.constants.FieldConstants.Reef.BranchHeight.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kElevator.*;
import static igknighters.subsystems.superStructure.SuperStructureConstants.kWrist.*;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import monologue.ProceduralStructGenerator;

public enum SuperStructureState implements StructSerializable {
  ScoreL4(MAX_HEIGHT, 25.0 * DEGREES_TO_RADIANS, 0.75),
  ScoreL3(L3.height + wristYOffset(L3.pitch) + 0.05, -L3.pitch, 1.0),
  ScoreL2(L2.height + wristYOffset(L2.pitch) + 0.05, -L2.pitch, 1.0),
  ScoreL1(L1.height, 10.0 * DEGREES_TO_RADIANS, 1.5),
  AlgaeL3(L3.height + 0.1, 0.0 * DEGREES_TO_RADIANS, 1.2),
  AlgaeL2(L2.height + 0.1, 0.0 * DEGREES_TO_RADIANS, 1.2),
  Stow(MIN_HEIGHT + STAGES[2].rangeOfMotion(), MAX_ANGLE, 2.0),
  Processor(20.0 * INCHES_TO_METERS, 0.0, 1.5),
  Net(MAX_HEIGHT, MAX_ANGLE, 1.0),
  IntakeHp(27.0 * INCHES_TO_METERS, -59.0 * DEGREES_TO_RADIANS, 0.8),
  AntiTilt(MIN_HEIGHT, MAX_ANGLE, 1.0);

  public final double elevatorMeters;
  public final double wristRads;
  public final double toleranceScalar;

  private static double wristYOffset(double theta) {
    return kWrist.LENGTH * Math.sin(theta);
  }

  SuperStructureState(double elevatorMeters, double wristRads, double toleranceScalar) {
    this.elevatorMeters = elevatorMeters;
    this.wristRads = wristRads;
    this.toleranceScalar = toleranceScalar;
  }

  public static final Struct<SuperStructureState> struct =
      ProceduralStructGenerator.genEnum(SuperStructureState.class);
}
