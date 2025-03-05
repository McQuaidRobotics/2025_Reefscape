package igknighters.commands;

import static igknighters.commands.Triggers.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    subsystemIdle(intake)
        .and(intake.isHolding(Holding.ALGAE))
        .onTrue(IntakeCommands.holdAlgae(intake));
    subsystemIdle(intake)
        .and(intake.isHolding(Holding.CORAL))
        .onTrue(IntakeCommands.holdCoral(intake));

    new Trigger(
            () -> {
              final Rotation3d robotRotation = swerve.getRotation();
              final double frontBackTiltLimit = Units.degreesToRadians(10.0);
              final double sideToSideTiltLimit = Units.degreesToRadians(7.0);
              return Math.abs(robotRotation.getX()) > frontBackTiltLimit
                  || Math.abs(robotRotation.getY()) > sideToSideTiltLimit;
            })
        .onTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.AntiTilt));
  }
}
