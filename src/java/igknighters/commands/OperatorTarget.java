package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.constants.FieldConstants.Reef;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.Set;
import java.util.function.Supplier;
import monologue.GlobalField;
import monologue.Logged;
import monologue.Monologue;
import monologue.ProceduralStructGenerator;
import monologue.ProceduralStructGenerator.IgnoreStructField;
import wpilibExt.AllianceSymmetry;

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
  @IgnoreStructField private final Subsystems subsystems;

  public OperatorTarget(Subsystems subsystems, Logged logger) {
    this.loggingNode = logger;
    this.subsystems = subsystems;
    logThis();
  }

  private void logThis() {
    loggingNode.log("OperatorTarget", this);
    GlobalField.setObject("OperatorTarget", targetLocation());
  }

  public Trigger isUpdated() {
    return new Trigger(() -> wasUpdated);
  }

  public Trigger hasTarget() {
    return new Trigger(() -> hasTarget);
  }

  public Trigger superStructureAtSetpoint() {
    return new Trigger(() -> !superStructureState.equals(SuperStructureState.Stow))
        .and(SuperStructureCommands.isAt(subsystems.superStructure, superStructureState));
  }

  public Pose2d targetLocation() {
    double backoffDist = (kRobotIntrinsics.CHASSIS_WIDTH / 2.0)
      + (5.0 * Conv.INCHES_TO_METERS);
    var ret =
        switch (faceSubLocation) {
          case LEFT -> side.alignScoreLeft(backoffDist, subsystems.intake.gamepieceYOffset());
          case RIGHT -> side.alignScoreRight(backoffDist, subsystems.intake.gamepieceYOffset());
          case CENTER -> side.alignScoreCenter(backoffDist, subsystems.intake.gamepieceYOffset());
        };
    if (AllianceSymmetry.isBlue()) {
      return ret;
    } else {
      return AllianceSymmetry.flip(ret);
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

  private Trigger isNearPose(Localizer localizer, Translation2d translation, double dist) {
    return new Trigger(
        () -> Monologue.log("offset", localizer.translation().getDistance(translation)) < dist);
  }

  public Command gotoTargetCmd(Localizer localizer) {
    Supplier<Command> c =
        () ->
            Commands.parallel(
                SwerveCommands.moveTo(
                    subsystems.swerve,
                    localizer,
                    targetLocation(),
                    PathObstacles.fromReefSide(side)),
                Commands.sequence(
                    Commands.waitUntil(
                        isNearPose(localizer, targetLocation().getTranslation(), 2.0)),
                    SuperStructureCommands.holdAt(
                            subsystems.superStructure,
                            superStructureState.minHeight(SuperStructureState.ScoreL3))
                        .until(isNearPose(localizer, targetLocation().getTranslation(), 0.1)),
                    SuperStructureCommands.holdAt(subsystems.superStructure, superStructureState)));
    return makeRefreshableCmd(c, subsystems.swerve, subsystems.superStructure);
  }

  public Command gotoSuperStructureTargetCmd() {
    Supplier<Command> c =
        () -> SuperStructureCommands.holdAt(subsystems.superStructure, superStructureState);
    return makeRefreshableCmd(c, subsystems.superStructure);
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
