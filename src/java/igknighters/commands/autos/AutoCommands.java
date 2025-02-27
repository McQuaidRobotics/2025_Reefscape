package igknighters.commands.autos;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.commands.SuperStructureCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.Holding;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import java.util.function.BooleanSupplier;
import monologue.Monologue;

public class AutoCommands {
  protected final Localizer localizer;
  protected final Swerve swerve;
  protected final Vision vision;
  protected final Led led;
  protected final SuperStructure superStructure;
  protected final Intake intake;

  private static final double timeBeforeIntakeMove, timeBeforeL4Move;

  static {
    final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1.8, 73.8));
    final var stowState = new TrapezoidProfile.State(SuperStructureState.Stow.elevatorMeters, 0.0);
    final var intakeStake =
        new TrapezoidProfile.State(SuperStructureState.IntakeHp.elevatorMeters, 0.0);
    final var l4State = new TrapezoidProfile.State(SuperStructureState.ScoreL4.elevatorMeters, 0.0);

    profile.calculate(0.02, stowState, l4State);
    timeBeforeL4Move = profile.timeLeftUntil(SuperStructureState.ScoreL3.elevatorMeters);
    System.out.println("Time before L4 move: " + timeBeforeL4Move);
    System.out.println("Total Move Time: " + profile.totalTime());
    System.out.println("Time after stop: " + (profile.totalTime() - timeBeforeL4Move));
    profile.calculate(0.01, stowState, intakeStake);
    timeBeforeIntakeMove = profile.totalTime();
  }

  protected AutoCommands(Subsystems subsystems, Localizer localizer) {
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
    return Commands.sequence(Commands.waitUntil(intake.isHolding(Holding.CORAL)), traj.cmd())
        .withName("AfterIntake" + traj.getRawTrajectory().name());
  }

  public Command afterScore(AutoTrajectory traj) {
    return Commands.sequence(Commands.waitUntil(intake.isHolding(Holding.NONE)), traj.cmd())
        .withName("AfterScore" + traj.getRawTrajectory().name());
  }

  public void orchestrateSuperstructure(AutoTrajectory traj) {
    final BooleanSupplier holdingAlgae = intake.isHolding(Holding.ALGAE);
    traj.active()
        .onTrue(
            SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow, holdingAlgae));
    traj.atTimeBeforeEnd(timeBeforeL4Move)
        .onTrue(
            SuperStructureCommands.holdAt(
                superStructure, SuperStructureState.ScoreL4, holdingAlgae));
  }

  public void orchestrateIntake(AutoTrajectory traj) {
    final BooleanSupplier holdingAlgae = intake.isHolding(Holding.ALGAE);
    traj.active()
        .onTrue(
            SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow, holdingAlgae));
    traj.atTimeBeforeEnd(timeBeforeIntakeMove * 1.2)
        .onTrue(
            SuperStructureCommands.holdAt(
                superStructure, SuperStructureState.IntakeHp, holdingAlgae));
  }
}
