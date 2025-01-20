package igknighters.commands.superStructure;

import java.io.File;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.constants.ConstValues;
import igknighters.constants.FieldConstants;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorReal;
import igknighters.subsystems.superStructure.Intake.Intake;
import igknighters.subsystems.superStructure.Intake.IntakeReal;
import igknighters.subsystems.superStructure.Wrist.Wrist;
import igknighters.subsystems.superStructure.Wrist.WristReal;

public class SuperStructureCommands {
  private final ElevatorReal elevator;
  private final WristReal wrist;
  private final IntakeReal intake;
  public SuperStructureCommands(ElevatorReal elevator, WristReal wrist, IntakeReal intake) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;
  }
  public Command superStructureGoToL4() {
    return Commands.parallel(
      elevator.gotoPosition(FieldConstants.ReefHeight.L4.height),
      wrist.goToPosition(ConstValues.Conv.DEGREES_TO_RADIANS * FieldConstants.ReefHeight.L4.pitch)
    );
  }
}
