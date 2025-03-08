package igknighters.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;

public class SubsystemTriggers {
  @SuppressWarnings("unused")
  public static void setupTriggers(Subsystems subsystems) {
    final Swerve swerve = subsystems.swerve;
    final Vision vision = subsystems.vision;
    final Led led = subsystems.led;
    final SuperStructure superStructure = subsystems.superStructure;
    final Intake intake = subsystems.intake;

    RobotModeTriggers.disabled()
        .negate()
        .and(IntakeCommands.isHolding(intake, Intake.Holding.CORAL))
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
