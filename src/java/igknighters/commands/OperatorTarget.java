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
import igknighters.commands.SuperStructureCommands.MoveOrder;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.constants.FieldConstants.Reef;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.Set;
import java.util.function.Supplier;
import monologue.GlobalField;
import monologue.Logged;
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

  public Trigger targeting(SuperStructureState state) {
    return new Trigger(() -> state == superStructureState);
  }

  public Trigger targeting(FaceSubLocation state) {
    return new Trigger(() -> state == faceSubLocation);
  }

  public Pose2d targetLocation() {
    double backoffDist = (kRobotIntrinsics.CHASSIS_WIDTH / 2.0) + (5.0 * Conv.INCHES_TO_METERS);
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
    if (!this.side.equals(side)) {
      wasUpdated = true;
    }
    this.side = side;
    hasTarget = true;
    logThis();
  }

  public void updateScoring(
      FaceSubLocation faceSubLocation, SuperStructureState superStructureState) {
    if (!this.faceSubLocation.equals(faceSubLocation)
        || !this.superStructureState.equals(superStructureState)) {
      wasUpdated = true;
    }
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
    return new Trigger(() -> localizer.translation().getDistance(translation) < dist);
  }

  private Trigger isSlowerThan(double speed) {
    return new Trigger(() -> subsystems.swerve.getFieldSpeeds().magnitude() < speed);
  }

  public Trigger wantsAlgae() {
    return new Trigger(
        () ->
            superStructureState.equals(SuperStructureState.AlgaeFloor)
                || superStructureState.equals(SuperStructureState.AlgaeL2)
                || superStructureState.equals(SuperStructureState.AlgaeL3));
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
                            superStructureState.minHeight(SuperStructureState.ScoreStaged))
                        .until(
                            isNearPose(localizer, targetLocation().getTranslation(), 0.04)
                                .and(isSlowerThan(0.4))),
                    SuperStructureCommands.holdAt(
                        subsystems.superStructure, superStructureState, MoveOrder.ELEVATOR_FIRST)));
    return makeRefreshableCmd(c, subsystems.swerve, subsystems.superStructure)
        .unless(wantsAlgae())
        .withName("TeleopAlignFull");
  }

  public Command gotoSuperStructureTargetCmd() {
    Supplier<Command> c =
        () -> SuperStructureCommands.holdAt(subsystems.superStructure, superStructureState);
    return makeRefreshableCmd(c, subsystems.superStructure).withName("TeleopSuperStructureAlign");
  }

  public Command updateTargetCmd(
      FaceSubLocation faceSubLocation, SuperStructureState superStructureState, Led led) {
    return led.runOnce(() -> updateScoring(faceSubLocation, superStructureState))
        .ignoringDisable(true)
        .withName("UpdateTargetScoring");
  }

  public Command updateTargetCmd(Reef.Side side) {
    return Commands.runOnce(() -> updateSide(side))
        .ignoringDisable(true)
        .withName("UpdateTargetFace");
  }

  public Command clearTargetCmd() {
    return Commands.runOnce(() -> hasTarget = false).ignoringDisable(true).withName("ClearTarget");
  }

  public static final Struct<OperatorTarget> struct =
      ProceduralStructGenerator.genObjectNoUnpack(OperatorTarget.class);
}
