package igknighters.commands;

import static igknighters.commands.Triggers.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
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

    final Trigger atAlgaeState =
        SuperStructureCommands.isAt(superStructure, SuperStructureState.AlgaeL3, 1.45)
            .or(SuperStructureCommands.isAt(superStructure, SuperStructureState.AlgaeL2, 1.45));

    subsystemIdle(intake)
        .and(intake.isHolding(Holding.ALGAE))
        .onTrue(IntakeCommands.runCurrent(intake, -80.0).withName("HoldAlgae"));
    subsystemIdle(intake)
        .and(intake.isHolding(Holding.CORAL))
        .onTrue(
            Commands.sequence(
                    IntakeCommands.runCurrent(intake, -40.0),
                    IntakeCommands.runVoltage(intake, 1.0).withTimeout(0.12),
                    IntakeCommands.runCurrent(intake, -20.0))
                .withName("HoldCoral"));

    new Trigger(
            () -> {
              final Rotation3d robotRotation = swerve.getRotation();
              final double frontBackTiltLimit = Units.degreesToRadians(10.0);
              final double sideToSideTiltLimit = Units.degreesToRadians(7.0);
              return Math.abs(robotRotation.getX()) > frontBackTiltLimit
                  || Math.abs(robotRotation.getY()) > sideToSideTiltLimit;
            })
        .onTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.AntiTilt));

    atAlgaeState
        .and(subsystemIdle(intake))
        .onTrue(IntakeCommands.intakeAlgae(intake).until(atAlgaeState.negate()));
  }
}
