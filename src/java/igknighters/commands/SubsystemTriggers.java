package igknighters.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.LEDCommands.LEDSection;
import igknighters.commands.OperatorTarget.FaceSubLocation;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import java.util.Set;

public class SubsystemTriggers {
  static int interpolateHeight(double elevatorHeight) {
    double min = SuperStructureState.ScoreL1.elevatorMeters;
    double max = SuperStructureState.ScoreL4.elevatorMeters;
    double t = (elevatorHeight - min) / (max - min);
    return (int) (36.0 * (0.25 + (t * 0.75)));
  }

  @SuppressWarnings("unused")
  public static void setupTriggers(Subsystems subsystems, OperatorTarget target) {
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

    final Trigger ledIdle = Triggers.subsystemIdle(led);

    ledIdle
        .and(target.hasTarget())
        .onTrue(
            Commands.defer(
                () ->
                    Commands.sequence(
                        LEDCommands.runSplitWithLEDSection(
                                led,
                                new LEDSection(
                                    0,
                                    0,
                                    LedUtil.makeFlash(0, 0, 255, .3),
                                    interpolateHeight(target.superStructureState().elevatorMeters),
                                    "FLASHY Blue Elevator Height Left"),
                                new LEDSection(
                                    1,
                                    0,
                                    LEDPattern.solid(Color.kGreen),
                                    interpolateHeight(target.superStructureState().elevatorMeters),
                                    "blue solid on right"))
                            .onlyIf(target.targeting(FaceSubLocation.LEFT)),
                      LEDCommands.runSplitWithLEDSection(
                        led,
                        new LEDSection(
                            1,
                            0,
                            LedUtil.makeFlash(0, 0, 255, .3),
                            interpolateHeight(target.superStructureState().elevatorMeters),
                            "FLASHY Blue Elevator Height CENTER"),
                        new LEDSection(
                            0,
                            0,
                            LedUtil.makeFlash(0, 0, 255, .3),
                            interpolateHeight(target.superStructureState().elevatorMeters),
                            "FLSHY BLUE ELEVATOR HEIGHT CENTER"))
                          .onlyIf(target.targeting(FaceSubLocation.CENTER)),
                      LEDCommands.runSplitWithLEDSection(
                        led,
                        new LEDSection(
                            1,
                            0,
                            LedUtil.makeFlash(0, 0, 255, .3),
                            interpolateHeight(target.superStructureState().elevatorMeters),
                            "FLASHY Blue Elevator Height right"),
                        new LEDSection(
                            0,
                            0,
                            LEDPattern.solid(Color.kGreen),
                            interpolateHeight(target.superStructureState().elevatorMeters),
                            "blue solid on left"))
                          .onlyIf(target.targeting(FaceSubLocation.RIGHT))),
                Set.of(led)));
  }
}
