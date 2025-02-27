package igknighters.commands.autos;

import static igknighters.commands.autos.Waypoints.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.subsystems.Subsystems;

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

  public Command test() {
    return newAuto("test")
        .addTrajectories(StartingOutside, FarLeft_L, Intake, FarLeft_R, Intake, CloseLeft_R, Intake)
        .build();
  }
}
