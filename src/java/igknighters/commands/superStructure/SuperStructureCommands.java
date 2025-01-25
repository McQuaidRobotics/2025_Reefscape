package igknighters.commands.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.FieldConstants.ReefHeight;
import igknighters.subsystems.superStructure.SuperStructure;

public class SuperStructureCommands {
  private final SuperStructure superStructure;

  public SuperStructureCommands(SuperStructure superstructure) {
    this.superStructure = superstructure;
  }

  public Command superStructureGoTo(ReefHeight reefHeight) {
    return superStructure
        .run(() -> superStructure.gotoPosition(reefHeight.height, reefHeight.pitch))
        .until(
            () -> superStructure.superStructureIsAt(reefHeight.height, reefHeight.pitch, 0.1, 0.1));
  }
}
