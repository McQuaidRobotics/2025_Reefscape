package igknighters.commands.superStructure;

import java.io.File;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.constants.ConstValues;
import igknighters.constants.FieldConstants;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorReal;
import igknighters.subsystems.superStructure.Intake.Intake;
import igknighters.subsystems.superStructure.Intake.IntakeReal;
import igknighters.subsystems.superStructure.Wrist.Wrist;
import igknighters.subsystems.superStructure.Wrist.WristReal;

public class SuperStructureCommands {
  private final SuperStructure superStructure;
  public SuperStructureCommands(SuperStructure superstructure) {
    this.superStructure = superstructure;
  }
  public Command superStructureGoToL4() {
    return superStructure.run(() -> superStructure.gotoPosition(FieldConstants.ReefHeight.L4.height, FieldConstants.ReefHeight.L4.pitch))
      .until(() -> superStructure.superStructureIsAt(FieldConstants.ReefHeight.L4.height, FieldConstants.ReefHeight.L4.pitch, 0.1, 0.1));
    
  }
}
