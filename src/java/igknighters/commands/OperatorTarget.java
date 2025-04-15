package igknighters.commands;

import static igknighters.commands.SwerveCommands.isSlowerThan;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.SuperStructureCommands.MoveOrder;
import igknighters.commands.teleop.TeleopSwerveTraditionalCmd;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kLed;
import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.constants.FieldConstants.FaceSubLocation;
import igknighters.constants.FieldConstants.Reef;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.controllers.DriverController;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.EnumMap;
import java.util.Set;
import java.util.function.Supplier;
import monologue.GlobalField;
import monologue.Logged;
import monologue.ProceduralStructGenerator;
import monologue.ProceduralStructGenerator.IgnoreStructField;
import wpilibExt.AllianceSymmetry;

public class OperatorTarget implements StructSerializable {
  private static final EnumMap<SuperStructureState, SuperStructureState> stagedStateMap =
      new EnumMap<>(SuperStructureState.class) {
        {
          put(SuperStructureState.ScoreL4, SuperStructureState.StagedL4);
          put(SuperStructureState.ScoreL3, SuperStructureState.ScoreL3);
          put(SuperStructureState.ScoreL2, SuperStructureState.ScoreL2);
        }
      };

  private static final EnumMap<Reef.Side, SuperStructureState> algaeHeightMap =
      new EnumMap<>(Reef.Side.class) {
        {
          put(Reef.Side.CLOSE_MID, SuperStructureState.AlgaeL3);
          put(Reef.Side.FAR_RIGHT, SuperStructureState.AlgaeL3);
          put(Reef.Side.FAR_LEFT, SuperStructureState.AlgaeL3);
          put(Reef.Side.FAR_MID, SuperStructureState.AlgaeL2);
          put(Reef.Side.CLOSE_LEFT, SuperStructureState.AlgaeL2);
          put(Reef.Side.CLOSE_RIGHT, SuperStructureState.AlgaeL2);
        }
      };

  private static final Transform2d ALGAE_BACKOFF = new Transform2d(-0.3, 0, Rotation2d.kZero);

  private boolean wasUpdated = false;
  private boolean hasTarget = false;
  private FaceSubLocation faceSubLocation = FaceSubLocation.CENTER;
  private Reef.Side side = Reef.Side.CLOSE_MID;
  private SuperStructureState superStructureState = SuperStructureState.Stow;

  @IgnoreStructField private final Localizer localizer;
  @IgnoreStructField private final Subsystems subsystems;
  @IgnoreStructField private final Logged loggingNode;

  public OperatorTarget(Localizer localizer, Subsystems subsystems, Logged logger) {
    this.localizer = localizer;
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

  public Trigger wantsAlgae() {
    return new Trigger(
        () ->
            superStructureState.equals(SuperStructureState.AlgaeFloor)
                || superStructureState.equals(SuperStructureState.AlgaeL2)
                || superStructureState.equals(SuperStructureState.AlgaeL3));
  }

  private Pose2d targetLocation() {
    Pose2d ret;
    if (!wantsAlgae().getAsBoolean()) {
      double backoffDist = (kRobotIntrinsics.CHASSIS_WIDTH / 2.0);
      if (!superStructureState.equals(SuperStructureState.ScoreL1)) {
        backoffDist += 5.0 * Conv.INCHES_TO_METERS;
      }
      ret =
          switch (faceSubLocation) {
            case LEFT -> side.alignScoreLeft(backoffDist, subsystems.intake.gamepieceYOffset());
            case RIGHT -> side.alignScoreRight(backoffDist, subsystems.intake.gamepieceYOffset());
            case CENTER -> side.alignScoreCenter(backoffDist, subsystems.intake.gamepieceYOffset());
          };
    } else {
      ret = side.alignScoreCenter(kRobotIntrinsics.CHASSIS_WIDTH / 2.0, 0);
    }
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

  private Trigger nearAndSlow(Pose2d targetLocation, double dist, double speed) {
    return localizer
        .near(targetLocation.getTranslation(), dist)
        .and(isSlowerThan(subsystems.swerve, speed));
  }

  private Command gotoTargetCmdScoreComponent(Pose2d target) {
    final MoveOrder preferredMoveOrder =
        superStructureState.equals(SuperStructureState.ScoreL4)
            ? new MoveOrder.ElevatorFirst(5.5 * Conv.INCHES_TO_METERS)
            : MoveOrder.SIMULTANEOUS;
    final SuperStructureState stagedState =
        stagedStateMap.getOrDefault(superStructureState, superStructureState);
    Command coral =
        Commands.sequence(
            SuperStructureCommands.holdAt(subsystems.superStructure, SuperStructureState.Stow)
                .until(localizer.near(target.getTranslation(), 1.45))
                .unless(targeting(SuperStructureState.ScoreL1)),
            SuperStructureCommands.holdAt(
                    subsystems.superStructure, superStructureState.minHeight(stagedState))
                .until(nearAndSlow(target, 0.04, 0.4)),
            SuperStructureCommands.holdAt(
                subsystems.superStructure, superStructureState, preferredMoveOrder));
    Command algae =
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  this.superStructureState = algaeHeightMap.getOrDefault(side, superStructureState);
                }),
            SuperStructureCommands.holdAt(
                subsystems.superStructure, algaeHeightMap.getOrDefault(side, superStructureState)));
    return Commands.either(algae, coral, wantsAlgae());
  }

  private Command gotoTargetCmdLineupComponent(Pose2d target, DriverController controller) {
    final Command lineup =
        SwerveCommands.lineupReef(
            subsystems.swerve, localizer, target, PathObstacles.fromReefSide(side));

    // if scoring on l1 or intaking algae allow the driver to
    // drive away without moving the superstructure.
    // this is needed due to descoring L1 and hitting algae on reef.
    if (superStructureState.equals(SuperStructureState.ScoreL1)
        || superStructureState.equals(SuperStructureState.AlgaeL2)
        || superStructureState.equals(SuperStructureState.AlgaeL3)) {
      final Command lineupBackedUp =
          SwerveCommands.lineupReef(
              subsystems.swerve,
              localizer,
              target.plus(ALGAE_BACKOFF),
              PathObstacles.fromReefSide(side));
      return Commands.sequence(
          lineupBackedUp.until(superStructureAtSetpoint()),
          lineup.until(nearAndSlow(target, 0.1, 0.05)),
          new TeleopSwerveTraditionalCmd(subsystems.swerve, controller));
    } else {
      return lineup;
    }
  }

  public Command gotoTargetCmd(DriverController controller) {
    final Supplier<Command> coral =
        () -> {
          final var targetLocation = targetLocation();
          return Commands.parallel(
              gotoTargetCmdScoreComponent(targetLocation),
              gotoTargetCmdLineupComponent(targetLocation, controller),
              Commands.sequence(
                  LEDCommands.run(subsystems.led, LedUtil.makeBounce(kLed.TargetingColor, 1.0))
                      .until(
                          nearAndSlow(targetLocation, 0.04, 0.2)
                              .and(
                                  SuperStructureCommands.isAt(
                                      subsystems.superStructure, superStructureState))),
                  new ScheduleCommand(
                          IntakeCommands.expel(
                              subsystems.intake,
                              () -> superStructureState.equals(SuperStructureState.ScoreL1)))
                      .onlyIf(
                          () ->
                              superStructureState.equals(SuperStructureState.ScoreL1)
                                  || superStructureState.equals(SuperStructureState.ScoreL2)
                                  || superStructureState.equals(SuperStructureState.ScoreL3)),
                  LEDCommands.run(subsystems.led, LedUtil.makeFlash(Color.kSnow, 0.25))));
        };
    return makeRefreshableCmd(coral, subsystems.swerve, subsystems.superStructure)
        .unless(hasTarget().negate())
        .unless(() -> superStructureState.equals(SuperStructureState.AlgaeFloor))
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
