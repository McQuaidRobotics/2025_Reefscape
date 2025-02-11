package igknighters.commands;

import static igknighters.commands.Triggers.*;

import igknighters.commands.intake.IntakeCommands;
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
        .onTrue(
            IntakeCommands.runTorque(intake, 0.45) // roughly 25 amps at stall
            );
    subsystemIdle(intake)
        .and(intake.isHolding(Holding.CORAL))
        .onTrue(
            IntakeCommands.runTorque(intake, 0.2) // roughly 10 amps at stall
            );
  }
}
