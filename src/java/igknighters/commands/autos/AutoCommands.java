package igknighters.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.commands.IntakeCommands;
import igknighters.commands.SuperStructureCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.Holding;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import java.util.function.BooleanSupplier;
import monologue.Monologue;

public class AutoCommands {

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
        new TrapezoidProfile.State(SuperStructureState.IntakeHp.elevatorMeters, 0.0);
    final var l4State = new TrapezoidProfile.State(SuperStructureState.ScoreL4.elevatorMeters, 0.0);

    profile.calculate(0.01, stowState, l4State);
    timeBeforeL4Move = profile.timeLeftUntil(SuperStructureState.ScoreL3.elevatorMeters);
    System.out.println("Time before L4 move: " + timeBeforeL4Move);
    System.out.println("Total Move Time: " + profile.totalTime());
    System.out.println("Time after stop: " + (profile.totalTime() - timeBeforeL4Move));
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
        logAutoEvent(this.getName(), "started");
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        super.end(interrupted);
        logAutoEvent(this.getName(), "ended");
      }
    };
  }

  public Command afterIntake(AutoTrajectory traj) {
    return loggedCmd(
        Commands.sequence(Commands.waitUntil(intake.isHolding(Holding.CORAL)), traj.cmd())
            .withName("AfterIntake" + traj.getRawTrajectory().name()));
  }

  public Command afterScore(AutoTrajectory traj) {
    return loggedCmd(
        Commands.sequence(Commands.waitUntil(intake.isHolding(Holding.NONE)), traj.cmd())
            .withName("AfterScore" + traj.getRawTrajectory().name()));
  }

  public void orchestrateSuperstructure(AutoTrajectory traj) {
    final BooleanSupplier holdingAlgae = intake.isHolding(Holding.ALGAE);
    traj.active()
        .onTrue(
            loggedCmd(
                SuperStructureCommands.holdAt(
                    superStructure, SuperStructureState.Stow, holdingAlgae)));
    traj.atTimeBeforeEnd(timeBeforeL4Move)
        .onTrue(
            loggedCmd(
                SuperStructureCommands.holdAt(
                    superStructure, SuperStructureState.ScoreL4, holdingAlgae)));
  }

  public void orchestrateIntake(AutoTrajectory traj) {
    final BooleanSupplier holdingAlgae = intake.isHolding(Holding.ALGAE);
    traj.active()
        .onTrue(
            loggedCmd(
                SuperStructureCommands.holdAt(
                    superStructure, SuperStructureState.Stow, holdingAlgae)));
    traj.atTimeBeforeEnd(timeBeforeIntakeMove * 1.2)
        .onTrue(
            loggedCmd(
                SuperStructureCommands.holdAt(
                    superStructure, SuperStructureState.IntakeHp, holdingAlgae)));
  }

  protected class ReefscapeAuto {
    private final AutoRoutine routine;
    private final ParallelCommandGroup headCommand = new ParallelCommandGroup();
    private final SequentialCommandGroup bodyCommand = new SequentialCommandGroup();
    private Trigger scorePathDone;
    private boolean trajectoryBeenAdded = false;

    private ReefscapeAuto(AutoRoutine routine) {
      this.routine = routine;
      scorePathDone = new Trigger(routine.loop(), () -> false);
    }

    public ReefscapeAuto addScoringTrajectory(Waypoints start, Waypoints end) {
      final AutoTrajectory traj = routine.trajectory(start.to(end));
      if (!trajectoryBeenAdded) {
        trajectoryBeenAdded = true;
        headCommand.addCommands(traj.resetOdometry(), SuperStructureCommands.home(superStructure));
      }
      bodyCommand.addCommands(afterIntake(traj));
      scorePathDone = scorePathDone.or(traj.recentlyDone());
      return this;
    }

    public ReefscapeAuto addIntakeTrajectory(Waypoints start, Waypoints end) {
      final AutoTrajectory traj = routine.trajectory(start.to(end));
      if (!trajectoryBeenAdded) {
        trajectoryBeenAdded = true;
        headCommand.addCommands(traj.resetOdometry(), SuperStructureCommands.home(superStructure));
      }
      bodyCommand.addCommands(afterScore(traj));
      return this;
    }

    public ReefscapeAuto addTrajectories(Waypoints... waypoints) {
      for (int i = 0; i < waypoints.length - 2; i++) {
        addScoringTrajectory(waypoints[i], waypoints[i + 1]);
        addIntakeTrajectory(waypoints[i + 1], waypoints[i + 2]);
      }
      return this;
    }

    public Command build() {
      scorePathDone
          .and(SuperStructureCommands.isAt(superStructure, SuperStructureState.ScoreL4))
          .onTrue(IntakeCommands.expel(intake));
      routine.active().onTrue(headCommand.withName(routine.name() + "_AutoHead"));
      routine
          .active()
          .and(superStructure::isHomed)
          .onTrue(bodyCommand.withName(routine.name() + "_AutoBody"));
      return routine.cmd();
    }
  }

  protected ReefscapeAuto newAuto(String name) {
    return new ReefscapeAuto(factory.newRoutine(name));
  }
}
