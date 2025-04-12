package igknighters.commands.autos;

import static igknighters.commands.autos.Waypoints.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;
import wpilibExt.Speeds;

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

  @FunctionalInterface
  public interface DualSideAuto {
    Command generate(boolean leftSide);
  }

  public static void addCmd(AutoChooser chooser, String name, DualSideAuto auto) {
    chooser.addCmd(name + " Left", () -> auto.generate(true));
    chooser.addCmd(name + " Right", () -> auto.generate(false));
  }

  public Command threePieceL4(boolean leftSide) {
    return newAuto("threePieceL4", leftSide)
        .addTrajectories(
            StartingOutside, FarLeft_R, Intake, CloseLeft_L, Intake, CloseLeft_R, Intake, FarLeft_L)
        .build();
  }

  public Command threePieceL4WithPush(boolean leftSide) {
    return Commands.sequence(
        new ProxyCommand(
            SwerveCommands.drive(swerve, Speeds.fromRobotRelative(-2.5, 0, 0)).withTimeout(0.25)),
        threePieceL4(leftSide));
  }

  public Command centralL4(boolean leftSide) {
    return newAuto("centralL4", leftSide).addTrajectories(StartingCenter, FarMid_R).build();
  }
}
