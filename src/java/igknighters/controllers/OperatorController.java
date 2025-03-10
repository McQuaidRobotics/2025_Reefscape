package igknighters.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.OperatorTarget;
import igknighters.commands.OperatorTarget.FaceSubLocation;
import igknighters.constants.FieldConstants.Reef;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.util.logging.BootupLogger;

public class OperatorController {
  // Define the bindings for the controller
  @SuppressWarnings("unused")
  public void bind(
      final Localizer localizer, final Subsystems subsystems, final OperatorTarget operatorTarget) {
    final var swerve = subsystems.swerve;
    final var vision = subsystems.vision;
    final var led = subsystems.led;

    // 1| 7|13|19
    // -----------
    // 2| 8|14|20
    // -----------
    // 3| 9|15|21
    // -----------
    // 4|10|16|22
    // -----------
    // 5|11|17|23
    // -----------
    // 6|12|18|24

    // ROW 1
    this.C1R1.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.LEFT, SuperStructureState.ScoreL4, led));
    this.C2R1.onTrue(Commands.none());
    this.C3R1.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.RIGHT, SuperStructureState.ScoreL4, led));
    this.C4R1.onTrue(Commands.none());

    // ROW 2
    this.C1R2.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.LEFT, SuperStructureState.ScoreL3, led));
    this.C2R2.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.CENTER, SuperStructureState.AlgaeL3, led));
    this.C3R2.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.RIGHT, SuperStructureState.ScoreL3, led));
    this.C4R2.onTrue(Commands.none());

    // ROW 3
    this.C1R3.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.LEFT, SuperStructureState.ScoreL2, led));
    this.C2R3.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.CENTER, SuperStructureState.AlgaeL2, led));
    this.C3R3.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.RIGHT, SuperStructureState.ScoreL2, led));
    this.C4R3.onTrue(Commands.none());

    // ROW 4
    this.C1R4.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.LEFT, SuperStructureState.ScoreL1, led));
    this.C2R4.onTrue(
        operatorTarget.updateTargetCmd(
            FaceSubLocation.CENTER, SuperStructureState.AlgaeFloor, led));
    this.C3R4.onTrue(
        operatorTarget.updateTargetCmd(FaceSubLocation.RIGHT, SuperStructureState.ScoreL1, led));
    this.C4R4.onTrue(operatorTarget.clearTargetCmd());

    // ROW 5
    this.C1R5.onTrue(operatorTarget.updateTargetCmd(Reef.Side.FAR_LEFT));
    this.C2R5.onTrue(operatorTarget.updateTargetCmd(Reef.Side.FAR_MID));
    this.C3R5.onTrue(operatorTarget.updateTargetCmd(Reef.Side.FAR_RIGHT));
    this.C4R5.onTrue(Commands.none());

    // ROW 6
    this.C1R6.onTrue(operatorTarget.updateTargetCmd(Reef.Side.CLOSE_LEFT));
    this.C2R6.onTrue(operatorTarget.updateTargetCmd(Reef.Side.CLOSE_MID));
    this.C3R6.onTrue(operatorTarget.updateTargetCmd(Reef.Side.CLOSE_RIGHT));
    this.C4R6.onTrue(Commands.none());

    // this.C1R1.onTrue(Commands.print("C1R1"));
    // this.C1R2.onTrue(Commands.print("C1R2"));
    // this.C1R3.onTrue(Commands.print("C1R3"));
    // this.C1R4.onTrue(Commands.print("C1R4"));
    // this.C1R5.onTrue(Commands.print("C1R5"));
    // this.C1R6.onTrue(Commands.print("C1R6"));
    // this.C2R1.onTrue(Commands.print("C2R1"));
    // this.C2R2.onTrue(Commands.print("C2R2"));
    // this.C2R3.onTrue(Commands.print("C2R3"));
    // this.C2R4.onTrue(Commands.print("C2R4"));
    // this.C2R5.onTrue(Commands.print("C2R5"));
    // this.C2R6.onTrue(Commands.print("C2R6"));
    // this.C3R1.onTrue(Commands.print("C3R1"));
    // this.C3R2.onTrue(Commands.print("C3R2"));
    // this.C3R3.onTrue(Commands.print("C3R3"));
    // this.C3R4.onTrue(Commands.print("C3R4"));
    // this.C3R5.onTrue(Commands.print("C3R5"));
    // this.C3R6.onTrue(Commands.print("C3R6"));
    // this.C4R1.onTrue(Commands.print("C4R1"));
    // this.C4R2.onTrue(Commands.print("C4R2"));
    // this.C4R3.onTrue(Commands.print("C4R3"));
    // this.C4R4.onTrue(Commands.print("C4R4"));
    // this.C4R5.onTrue(Commands.print("C4R5"));
    // this.C4R6.onTrue(Commands.print("C4R6"));
  }

  // Define the buttons on the controller

  private final CommandGenericHID controller;

  // make 4 columns and 6 rows
  protected final Trigger C1R1;
  protected final Trigger C1R2;
  protected final Trigger C1R3;
  protected final Trigger C1R4;
  protected final Trigger C1R5;
  protected final Trigger C1R6;
  protected final Trigger C2R1;
  protected final Trigger C2R2;
  protected final Trigger C2R3;
  protected final Trigger C2R4;
  protected final Trigger C2R5;
  protected final Trigger C2R6;
  protected final Trigger C3R1;
  protected final Trigger C3R2;
  protected final Trigger C3R3;
  protected final Trigger C3R4;
  protected final Trigger C3R5;
  protected final Trigger C3R6;
  protected final Trigger C4R1;
  protected final Trigger C4R2;
  protected final Trigger C4R3;
  protected final Trigger C4R4;
  protected final Trigger C4R5;
  protected final Trigger C4R6;

  private Trigger getButtonTrigger(int column, int row) {
    return controller.button(((column - 1) * 6) + row);
  }

  public OperatorController(int port) {
    controller = new CommandGenericHID(port);
    BootupLogger.bootupLog("Controller " + port + " initialized");
    C1R1 = getButtonTrigger(1, 1);
    C1R2 = getButtonTrigger(1, 2);
    C1R3 = getButtonTrigger(1, 3);
    C1R4 = getButtonTrigger(1, 4);
    C1R5 = getButtonTrigger(1, 5);
    C1R6 = getButtonTrigger(1, 6);
    C2R1 = getButtonTrigger(2, 1);
    C2R2 = getButtonTrigger(2, 2);
    C2R3 = getButtonTrigger(2, 3);
    C2R4 = getButtonTrigger(2, 4);
    C2R5 = getButtonTrigger(2, 5);
    C2R6 = getButtonTrigger(2, 6);
    C3R1 = getButtonTrigger(3, 1);
    C3R2 = getButtonTrigger(3, 2);
    C3R3 = getButtonTrigger(3, 3);
    C3R4 = getButtonTrigger(3, 4);
    C3R5 = getButtonTrigger(3, 5);
    C3R6 = getButtonTrigger(3, 6);
    C4R1 = getButtonTrigger(4, 1);
    C4R2 = getButtonTrigger(4, 2);
    C4R3 = getButtonTrigger(4, 3);
    C4R4 = getButtonTrigger(4, 4);
    C4R5 = getButtonTrigger(4, 5);
    C4R6 = getButtonTrigger(4, 6);
  }
}
