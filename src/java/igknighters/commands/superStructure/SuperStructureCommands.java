package igknighters.commands.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.superStructure.SuperStructure;

public class SuperStructureCommands {
  private final SuperStructure superStructure;

  public SuperStructureCommands(SuperStructure superstructure) {
    this.superStructure = superstructure;
  }

  public Command superStructureGoToL4() {
    return superStructure
        .run(
            () ->
                superStructure.gotoPosition(
                    FieldConstants.ReefHeight.L4.height, FieldConstants.ReefHeight.L4.pitch))
        .until(
            () ->
                superStructure.superStructureIsAt(
                    FieldConstants.ReefHeight.L4.height,
                    FieldConstants.ReefHeight.L4.pitch,
                    0.1,
                    0.1));
  }

  public Command superStructureGoToL3() {
    return superStructure
        .run(
            () ->
                superStructure.gotoPosition(
                    FieldConstants.ReefHeight.L3.height, FieldConstants.ReefHeight.L3.pitch))
        .until(
            () ->
                superStructure.superStructureIsAt(
                    FieldConstants.ReefHeight.L3.height,
                    FieldConstants.ReefHeight.L3.pitch,
                    0.1,
                    0.1));
  }

  public Command superStructureGoToL2() {
    return superStructure
        .run(
            () ->
                superStructure.gotoPosition(
                    FieldConstants.ReefHeight.L2.height, FieldConstants.ReefHeight.L2.pitch))
        .until(
            () ->
                superStructure.superStructureIsAt(
                    FieldConstants.ReefHeight.L2.height,
                    FieldConstants.ReefHeight.L2.pitch,
                    0.1,
                    0.1));
  }

  public Command superStructureGoToL1() {
    return superStructure
        .run(
            () ->
                superStructure.gotoPosition(
                    FieldConstants.ReefHeight.L1.height, FieldConstants.ReefHeight.L1.pitch))
        .until(
            () ->
                superStructure.superStructureIsAt(
                    FieldConstants.ReefHeight.L1.height,
                    FieldConstants.ReefHeight.L1.pitch,
                    0.1,
                    0.1));
  }
}
