package igknighters.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.commands.IntakeCommands;
import igknighters.commands.SuperStructureCommands;
import igknighters.commands.SuperStructureCommands.MoveOrder;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.Holding;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import monologue.GlobalField;
import monologue.Monologue;

public class AutoCommands {

  private static final AtomicInteger counter = new AtomicInteger();

  protected final AutoFactory factory;
  protected final Localizer localizer;
  protected final Swerve swerve;
  protected final Vision vision;
  protected final Led led;
  protected final SuperStructure superStructure;
  protected final Intake intake;

  private static final double timeBeforeIntakeMove, timeBeforeL4Move;

  static {
    final TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                kElevator.MAX_VELOCITY * kElevator.PULLEY_RADIUS,
                kElevator.MAX_ACCELERATION * kElevator.PULLEY_RADIUS));
    final var stowState = new TrapezoidProfile.State(SuperStructureState.Stow.elevatorMeters, 0.0);
    final var intakeStake =
        new TrapezoidProfile.State(SuperStructureState.IntakeHpClose.elevatorMeters, 0.0);
    final var l4State = new TrapezoidProfile.State(SuperStructureState.ScoreL4.elevatorMeters, 0.0);

    profile.calculate(0.01, stowState, l4State);
    timeBeforeL4Move = profile.timeLeftUntil(SuperStructureState.ScoreL3.elevatorMeters);
    // System.out.println("Time before L4 move: " + timeBeforeL4Move);
    // System.out.println("Total Move Time: " + profile.totalTime());
    // System.out.println("Time after stop: " + (profile.totalTime() - timeBeforeL4Move));
    profile.calculate(0.01, stowState, intakeStake);
    timeBeforeIntakeMove = profile.totalTime();
  }

  protected AutoCommands(AutoFactory factory, Subsystems subsystems, Localizer localizer) {
    this.factory = factory;
    this.localizer = localizer;
    swerve = subsystems.swerve;
    vision = subsystems.vision;
    led = subsystems.led;
    superStructure = subsystems.superStructure;
    intake = subsystems.intake;
  }

  protected void logAutoEvent(String name, String event) {
    String msg = "[Auto] Command " + name + " " + event;
    if (Robot.isDebug()) System.out.println(msg);
    Monologue.log("AutoEvent", msg);
  }

  protected Command loggedCmd(Command command) {
    return new WrapperCommand(command) {
      @Override
      public void initialize() {
        logAutoEvent(this.getName(), "Started");
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        super.end(interrupted);
        logAutoEvent(this.getName(), "Finished");
      }
    };
  }

  protected Trigger movingSlowerThan(Swerve swerve, double speed) {
    return new Trigger(() -> swerve.getRobotSpeeds().magnitude() < speed);
  }

  public Command afterIntake(AutoTrajectory traj) {
    return loggedCmd(
        Commands.sequence(
                Commands.waitUntil(IntakeCommands.isHolding(intake, Holding.CORAL)),
                traj.cmd(),
                SwerveCommands.stop(swerve))
            .withName("AfterIntake" + traj.getRawTrajectory().name()));
  }

  public Command afterScore(AutoTrajectory traj) {
    return loggedCmd(
        Commands.sequence(
                SwerveCommands.stop(swerve).until(IntakeCommands.isHolding(intake, Holding.NONE)),
                traj.cmd(),
                SwerveCommands.stop(swerve))
            .withName("AfterScore" + traj.getRawTrajectory().name()));
  }

  public void orchestrateScoring(AutoTrajectory traj) {
    traj.active()
        .onTrue(loggedCmd(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow)));
    traj.atTimeBeforeEnd(timeBeforeL4Move)
        .onTrue(
            loggedCmd(
                SuperStructureCommands.holdAt(
                    superStructure, SuperStructureState.ScoreL4, MoveOrder.ELEVATOR_FIRST)));
  }

  public void orchestrateIntake(AutoTrajectory traj) {
    traj.active()
        .onTrue(loggedCmd(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow)));
    traj.atTimeBeforeEnd(
            Math.min(timeBeforeIntakeMove * 1.2, traj.getRawTrajectory().getTotalTime() * 0.90))
        .onTrue(
            loggedCmd(
                SuperStructureCommands.holdAt(superStructure, SuperStructureState.IntakeHpClose)))
        .onTrue(loggedCmd(IntakeCommands.intakeCoral(intake)));
  }

  protected class ReefscapeAuto {
    private final AutoRoutine routine;
    private final ParallelCommandGroup headCommand = new ParallelCommandGroup();
    private final SequentialCommandGroup bodyCommand = new SequentialCommandGroup();
    private final boolean leftSide;
    private boolean trajectoryBeenAdded = false;

    private ReefscapeAuto(AutoRoutine routine, boolean leftSide) {
      this.routine = routine;
      this.leftSide = leftSide;
      headCommand.addCommands(loggedCmd(SuperStructureCommands.home(superStructure, false)));
    }

    private AutoTrajectory getTrajectory(Waypoints start, Waypoints end) {
      if (leftSide) {
        return routine.trajectory(start.to(end));
      } else {
        Trajectory<?> rawTraj = factory.cache().loadTrajectory(start.to(end)).orElseThrow();
        rawTraj = rawTraj.flipped(Flipper.MIRRORED_X);
        return routine.trajectory(rawTraj);
      }
    }

    private Command finishAlignment(AutoTrajectory trajectory) {
      if (trajectory.getFinalPose().isPresent()) {
        Supplier<Command> cmdSup =
            () -> {
              Pose2d finalPose = trajectory.getFinalPose().get();
              finalPose =
                  finalPose.plus(
                      new Transform2d(
                          0.0,
                          Monologue.log("gpOffset", -intake.gamepieceYOffset()),
                          Rotation2d.kZero));
              GlobalField.setObject("FinishAlignment" + counter.incrementAndGet(), finalPose);
              return loggedCmd(
                      SwerveCommands.moveToSimple(swerve, localizer, finalPose)
                          .until(
                              localizer
                                  .near(finalPose.getTranslation(), 0.04)
                                  .and(movingSlowerThan(swerve, 0.1)))
                          .withName("FinishAlignment"))
                  .andThen(SwerveCommands.stop(swerve));
            };
        return Commands.defer(cmdSup, Set.of(swerve));
      } else {
        DriverStation.reportError("womp womp", false);
        return Commands.none();
      }
    }

    public ReefscapeAuto addScoringTrajectory(Waypoints start, Waypoints end) {
      final AutoTrajectory traj = getTrajectory(start, end);
      if (!trajectoryBeenAdded) {
        trajectoryBeenAdded = true;
        headCommand.addCommands(traj.resetOdometry());
      }
      orchestrateScoring(traj);
      bodyCommand.addCommands(
          afterIntake(traj),
          finishAlignment(traj),
          loggedCmd(
              Commands.waitUntil(
                      SuperStructureCommands.isAt(superStructure, SuperStructureState.ScoreL4))
                  .withTimeout(1.0)
                  .withName("WaitForL4")),
          Commands.waitSeconds(0.1),
          new ScheduleCommand(loggedCmd(IntakeCommands.expel(intake, () -> false))),
          Commands.waitSeconds(0.25));
      return this;
    }

    public ReefscapeAuto addIntakeTrajectory(Waypoints start, Waypoints end) {
      final AutoTrajectory traj = getTrajectory(start, end);
      if (!trajectoryBeenAdded) {
        trajectoryBeenAdded = true;
        headCommand.addCommands(traj.resetOdometry());
      }
      orchestrateIntake(traj);
      bodyCommand.addCommands(
          afterScore(traj),
          Commands.sequence(
                  SwerveCommands.driveVolts(swerve, Rotation2d.kZero, 1.0).withTimeout(0.33),
                  Commands.repeatingSequence(SwerveCommands.stop(swerve)))
              .until(IntakeCommands.isHolding(intake, Holding.CORAL)));
      return this;
    }

    public ReefscapeAuto addTrajectories(Waypoints... waypoints) {
      boolean scoring = true;
      for (int i = 0; i < waypoints.length - 1; i++) {
        if (scoring) {
          // System.out.println(
          //     "Adding scoring trajectory " + waypoints[i] + " to " + waypoints[i + 1]);
          addScoringTrajectory(waypoints[i], waypoints[i + 1]);
        } else {
          // System.out.println(
          //     "Adding intake trajectory " + waypoints[i] + " to " + waypoints[i + 1]);
          addIntakeTrajectory(waypoints[i], waypoints[i + 1]);
        }
        scoring = !scoring;
      }
      return this;
    }

    public ReefscapeAuto addTrajectoriesMoveOnly(Waypoints... waypoints) {
      headCommand.addCommands(getTrajectory(waypoints[0], waypoints[1]).resetOdometry());
      bodyCommand.addCommands(
          new ScheduleCommand(
              loggedCmd(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow))));
      for (int i = 0; i < waypoints.length - 2; i += 2) {
        bodyCommand.addCommands(
            getTrajectory(waypoints[i], waypoints[i + 1]).cmd(),
            Commands.waitSeconds(3.0),
            getTrajectory(waypoints[i + 1], waypoints[i + 2]).cmd(),
            Commands.waitSeconds(3.0));
      }
      return this;
    }

    public Command build() {
      headCommand.addCommands(Commands.print(bodyCommand.getRequirements().toString()));
      bodyCommand.addCommands(
          new ScheduleCommand(
              loggedCmd(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow))));
      routine
          .active()
          .onTrue(
              headCommand
                  .andThen(new ScheduleCommand(bodyCommand.withName(routine.name() + "_AutoBody")))
                  .withName(routine.name() + "_AutoHead"));
      return routine.cmd();
    }
  }

  protected ReefscapeAuto newAuto(String name, boolean leftSide) {
    return new ReefscapeAuto(factory.newRoutine(name), leftSide);
  }
}
