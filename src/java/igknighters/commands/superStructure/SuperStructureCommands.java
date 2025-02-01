package igknighters.commands.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.Wrist.WristConstants;

public class SuperStructureCommands {

  public static Command moveTo(
      SuperStructure superStructure,
      double elevatorMeters,
      double wristRads,
      double toleranceScalar) {
    return superStructure
        .run(() -> superStructure.goTo(elevatorMeters, wristRads))
        .until(
            () ->
                superStructure.isAt(
                    elevatorMeters,
                    wristRads,
                    ElevatorConstants.DEFAULT_TOLERANCE * toleranceScalar,
                    WristConstants.DEFAULT_TOLERANCE * toleranceScalar));
  }
}
