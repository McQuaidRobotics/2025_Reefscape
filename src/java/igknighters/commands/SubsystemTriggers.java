package igknighters.commands;

import static igknighters.commands.Triggers.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.intake.IntakeCommands;
import igknighters.commands.superStructure.StateManager;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake.Holding;
import igknighters.subsystems.superStructure.SuperStructureState;

public class SubsystemTriggers {
  @SuppressWarnings("unused")
  public static void setupTriggers(Subsystems subsystems) {
    final var swerve = subsystems.swerve;
    final var vision = subsystems.vision;
    final var led = subsystems.led;
    final var superStructure = subsystems.superStructure;
    final var intake = subsystems.intake;

    final StateManager stateManager = new StateManager(superStructure);

    subsystemIdle(intake)
        .and(intake.isHolding(Holding.ALGAE))
        .onTrue(IntakeCommands.runCurrent(intake, 65.0));
    subsystemIdle(intake)
        .and(intake.isHolding(Holding.CORAL))
        .onTrue(IntakeCommands.runCurrent(intake, 20.0));
    new Trigger(
            () -> {
              final Rotation3d robotRotation = swerve.getRotation();
              final double tiltLimit = Units.degreesToRadians(17.5);
              return Math.abs(robotRotation.getX()) > tiltLimit
                  || Math.abs(robotRotation.getY()) > tiltLimit;
            })
        .onTrue(stateManager.holdAt(SuperStructureState.AntiTilt));
  }
}
