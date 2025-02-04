package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.superStructure.SuperStructureCommands;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.constants.FieldConstants.Reef;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import java.util.Set;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.ProceduralStructGenerator;
import monologue.ProceduralStructGenerator.IgnoreStructField;
import wpilibExt.AllianceFlipper;

public class OperatorTarget implements StructSerializable {
  public enum FaceSubLocation implements StructSerializable {
    LEFT,
    RIGHT,
    CENTER;

    public static final Struct<FaceSubLocation> struct =
        ProceduralStructGenerator.genEnum(FaceSubLocation.class);
  }

  private boolean wasUpdated = false;
  private boolean hasTarget = false;
  private FaceSubLocation faceSubLocation = FaceSubLocation.CENTER;
  private Reef.Side side = Reef.Side.CLOSE_MID;
  private SuperStructureState superStructureState = SuperStructureState.Stow;

  @IgnoreStructField private final Logged loggingNode;

  public OperatorTarget(Logged logger) {
    this.loggingNode = logger;
    logThis();
  }

  private void logThis() {
    loggingNode.log("OperatorTarget", this);
  }

  public Trigger isUpdated() {
    return new Trigger(() -> wasUpdated);
  }

  public Trigger hasTarget() {
    return new Trigger(() -> hasTarget);
  }

  public Pose2d targetLocation() {
    var ret =
        switch (faceSubLocation) {
          case LEFT -> side.scoreLeft(kRobotIntrinsics.CHASSIS_WIDTH / 1.9);
          case RIGHT -> side.scoreRight(kRobotIntrinsics.CHASSIS_WIDTH / 1.9);
          case CENTER -> side.scoreCenter(kRobotIntrinsics.CHASSIS_WIDTH / 1.9);
        };
    if (AllianceFlipper.isBlue()) {
      return ret;
    } else {
      return AllianceFlipper.flip(ret);
    }
  }

  public SuperStructureState superStructureState() {
    return superStructureState;
  }

  public void updateSide(Reef.Side side) {
    this.side = side;
    wasUpdated = true;
    hasTarget = true;
    logThis();
  }

  public void updateScoring(
      FaceSubLocation faceSubLocation, SuperStructureState superStructureState) {
    this.faceSubLocation = faceSubLocation;
    this.superStructureState = superStructureState;
    wasUpdated = true;
    hasTarget = true;
    logThis();
  }

  private Command makeRefreshableCmd(Supplier<Command> cmdSupplier, Subsystem... requirements) {
    return Commands.repeatingSequence(
        Commands.runOnce(() -> wasUpdated = false),
        Commands.defer(cmdSupplier, Set.of(requirements))
            .until(isUpdated())
            .unless(hasTarget().negate()));
  }

  public Command gotoTargetCmd(Swerve swerve, SuperStructure superStructure, Localizer localizer) {
    Supplier<Command> c =
        () ->
            Commands.parallel(
                Commands.print("Goto Target"),
                SwerveCommands.moveTo(
                    swerve, localizer, targetLocation(), PathObstacles.fromReefSide(side)),
                SuperStructureCommands.holdAt(superStructure, superStructureState));
    return makeRefreshableCmd(c, swerve, superStructure);
  }

  public Command gotoSuperStructureTargetCmd(SuperStructure superStructure) {
    Supplier<Command> c = () -> SuperStructureCommands.holdAt(superStructure, superStructureState);
    return makeRefreshableCmd(c, superStructure);
  }

  public Command updateTargetCmd(
      FaceSubLocation faceSubLocation, SuperStructureState superStructureState) {
    return Commands.runOnce(() -> updateScoring(faceSubLocation, superStructureState))
        .ignoringDisable(true);
  }

  public Command updateTargetCmd(Reef.Side side) {
    return Commands.runOnce(() -> updateSide(side)).ignoringDisable(true);
  }

  public Command clearTargetCmd() {
    return Commands.runOnce(() -> hasTarget = false).ignoringDisable(true);
  }

  public static final Struct<OperatorTarget> struct =
      ProceduralStructGenerator.genObjectNoUnpack(OperatorTarget.class);
}
