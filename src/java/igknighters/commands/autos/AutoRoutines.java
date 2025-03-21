package igknighters.commands.autos;

import static igknighters.commands.autos.Waypoints.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.commands.IntakeCommands;
import igknighters.commands.SuperStructureCommands;
import igknighters.commands.SuperStructureCommands.MoveOrder;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.Holding;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.function.Supplier;

public class AutoRoutines extends AutoCommands {

  public AutoRoutines(Subsystems subsystems, Localizer localizer, AutoFactory factory) {
    super(factory, subsystems, localizer);

    if (Robot.isSimulation()) {
      new Trigger(DriverStation::isAutonomousEnabled)
          .onTrue(
              Commands.waitSeconds(15.3)
                  .andThen(() -> DriverStationSim.setEnabled(false))
                  .withName("Simulated Auto Ender"));
    }
  }

  public Supplier<Command> trajTest(String trajName) {
    return () ->
        Commands.sequence(factory.resetOdometry(trajName), factory.trajectoryCmd(trajName));
  }

  public Command grabAlgaeFarMid_C() {
    return Commands.parallel(
        Commands.sequence(
            factory.resetOdometry("FarMid_LToFarMid_C"),
            factory.trajectoryCmd("FarMid_LToFarMid_C")),
        Commands.sequence(
            SuperStructureCommands.holdAt(
                superStructure, SuperStructureState.AlgaeL3, MoveOrder.ELEVATOR_FIRST),
            IntakeCommands.intakeAlgae(intake)));
  }

  public Command algaeFarMid_CToBarge_R() {
    return Commands.sequence(
        IntakeCommands.holdAlgae(intake),
        factory.resetOdometry("FarMid_CToBarge_R"),
        factory.trajectoryCmd("FarMid_LToFarMid_C"),
        Commands.parallel(
            Commands.sequence(
                SuperStructureCommands.moveTo(superStructure, SuperStructureState.Net_FLICKED),
                new ScheduleCommand(
                    SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow))),
            Commands.sequence(
                IntakeCommands.holdAlgae(intake).withTimeout(0.14),
                IntakeCommands.expel(intake).withTimeout(0.25))));
  }

  public Command algaeBarge(boolean leftSide, SuperStructure superStructure, Intake intake) {
    return newAuto("algaeBarge", leftSide)
        .addScoringTrajectory(StartingMiddle, FarMid_C)
        .build()
        .andThen(grabAlgaeFarMid_C().until(() -> intake.getHolding().equals(Holding.ALGAE)))
        .andThen(algaeFarMid_CToBarge_R().until(() -> intake.getHolding().equals(Holding.NONE)));
  }

  @FunctionalInterface
  public interface DualSideAuto {
    Command generate(boolean leftSide);
  }

  public static void addCmd(AutoChooser chooser, String name, DualSideAuto auto) {
    chooser.addCmd(name + " Left", () -> auto.generate(true));
    chooser.addCmd(name + " Right", () -> auto.generate(false));
  }

  public Command test(boolean leftSide) {
    return newAuto("test", leftSide)
        .addTrajectories(
            StartingOutside, FarLeft_R, Intake, CloseLeft_L, Intake, CloseLeft_R, Intake, FarLeft_L)
        .build();
  }

  public Command testMove(boolean leftSide) {
    return newAuto("testMove", leftSide)
        .addTrajectoriesMoveOnly(
            StartingOutside, FarLeft_R, Intake, CloseLeft_L, Intake, CloseLeft_R, Intake)
        .build();
  }
}
