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
import igknighters.constants.FieldConstants.FaceSubLocation;
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

  public FaceSubLocation faceSubLocation() {
    return faceSubLocation;
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

  public void updateSide(Reef.Side side, FaceSubLocation fsl) {
    if (!this.side.equals(side) || !this.faceSubLocation.equals(fsl)) {
      wasUpdated = true;
    }
    this.side = side;
    this.faceSubLocation = fsl;
    hasTarget = true;
    logThis();
  }

  public void updateScoring(SuperStructureState superStructureState) {
    if (!this.superStructureState.equals(superStructureState)) {
      wasUpdated = true;
    }
    this.superStructureState = superStructureState;
    logThis();
  }

  private Command makeRefreshableCmd(Supplier<Command> cmdSupplier, Subsystem... requirements) {
    return Commands.repeatingSequence(
        Commands.runOnce(() -> wasUpdated = false),
        Commands.defer(cmdSupplier, Set.of(requirements)).until(isUpdated()));
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
        () -> {
          SuperStructureState preferredStow;
          if (superStructureState.elevatorMeters > SuperStructureState.ScoreL3.elevatorMeters) {
            preferredStow = SuperStructureState.ScoreStagedHigh;
          } else if (superStructureState.elevatorMeters
              > SuperStructureState.ScoreL2.elevatorMeters) {
            preferredStow = SuperStructureState.Stow;
          } else {
            preferredStow = SuperStructureState.ScoreStagedLow;
          }
          final MoveOrder preferredMoveOrder =
              superStructureState.elevatorMeters > SuperStructureState.ScoreL3.elevatorMeters
                  ? MoveOrder.ELEVATOR_FIRST
                  : MoveOrder.SIMULTANEOUS;
          return Commands.parallel(
              SwerveCommands.lineupReef(
                  subsystems.swerve, localizer, targetLocation(), PathObstacles.fromReefSide(side)),
              Commands.sequence(
                  SuperStructureCommands.holdAt(subsystems.superStructure, SuperStructureState.Stow)
                      .until(isNearPose(localizer, targetLocation().getTranslation(), 2.0)),
                  SuperStructureCommands.holdAt(
                          subsystems.superStructure, superStructureState.minHeight(preferredStow))
                      .until(
                          isNearPose(localizer, targetLocation().getTranslation(), 0.04)
                              .and(isSlowerThan(0.4))),
                  SuperStructureCommands.holdAt(
                      subsystems.superStructure, superStructureState, preferredMoveOrder)));
        };
    return makeRefreshableCmd(c, subsystems.swerve, subsystems.superStructure)
        .unless(wantsAlgae())
        .unless(hasTarget().negate())
        .withName("TeleopAlignFull");
  }

  public Command gotoSuperStructureTargetCmd() {
    Supplier<Command> c =
        () -> SuperStructureCommands.holdAt(subsystems.superStructure, superStructureState);
    return makeRefreshableCmd(c, subsystems.superStructure).withName("TeleopSuperStructureAlign");
  }

  public Command updateTargetCmd(SuperStructureState superStructureState, Led led) {
    return led.runOnce(() -> updateScoring(superStructureState))
        .ignoringDisable(true)
        .withName("UpdateTargetScoring");
  }

  public Command updateTargetCmd(Reef.Side side, FaceSubLocation faceSubLocation) {
    return Commands.runOnce(() -> updateSide(side, faceSubLocation))
        .ignoringDisable(true)
        .withName("UpdateTargetFace");
  }

  public Command clearTargetCmd() {
    return Commands.runOnce(() -> hasTarget = false).ignoringDisable(true).withName("ClearTarget");
  }

  public static final Struct<OperatorTarget> struct =
      ProceduralStructGenerator.genObjectNoUnpack(OperatorTarget.class);
}
