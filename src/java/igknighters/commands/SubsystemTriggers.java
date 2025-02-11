package igknighters.commands;

import static igknighters.commands.Triggers.*;

import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake.Holding;

public class SubsystemTriggers {
  @SuppressWarnings("unused")
  public static void setupTriggers(Subsystems subsystems) {
    final var swerve = subsystems.swerve;
    final var vision = subsystems.vision;
    final var led = subsystems.led;
    final var superStructure = subsystems.superStructure;
    final var intake = subsystems.intake;

    subsystemIdle(intake)
        .and(intake.isHolding(Holding.ALGAE))
        .onTrue(IntakeCommands.runCurrent(intake, 25.0));
    subsystemIdle(intake)
        .and(intake.isHolding(Holding.CORAL))
        .onTrue(IntakeCommands.runCurrent(intake, 10.0));
    subsystemIdle(intake)
        .and(intake.isHolding(Holding.NONE))
        .onTrue(IntakeCommands.runCurrent(intake, 0.0));
  }
}
