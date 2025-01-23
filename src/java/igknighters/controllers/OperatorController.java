package igknighters.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.subsystems.Subsystems;
import igknighters.util.logging.BootupLogger;

public class OperatorController {
  // Define the bindings for the controller
  @SuppressWarnings("unused")
  public void bind(final Localizer localizer, final Subsystems subsystems) {
    final var swerve = subsystems.swerve;
    final var vision = subsystems.vision;
    final var led = subsystems.led;

    // ROW 1
    this.C1R1.onTrue(Commands.none());
    this.C2R1.onTrue(Commands.none());
    this.C3R1.onTrue(Commands.none());
    this.C4R1.onTrue(Commands.none());

    // ROW 2
    this.C1R2.onTrue(Commands.none());
    this.C2R2.onTrue(Commands.none());
    this.C3R2.onTrue(Commands.none());
    this.C4R2.onTrue(Commands.none());

    // ROW 3
    this.C1R3.onTrue(Commands.none());
    this.C2R3.onTrue(Commands.none());
    this.C3R3.onTrue(Commands.none());
    this.C4R3.onTrue(Commands.none());

    // ROW 4
    this.C1R4.onTrue(Commands.none());
    this.C2R4.onTrue(Commands.none());
    this.C3R4.onTrue(Commands.none());
    this.C4R4.onTrue(Commands.none());

    // ROW 5
    this.C1R5.onTrue(Commands.none());
    this.C2R5.onTrue(Commands.none());
    this.C3R5.onTrue(Commands.none());
    this.C4R5.onTrue(Commands.none());

    // ROW 6
    this.C1R6.onTrue(Commands.none());
    this.C2R6.onTrue(Commands.none());
    this.C3R6.onTrue(Commands.none());
    this.C4R6.onTrue(Commands.none());
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
