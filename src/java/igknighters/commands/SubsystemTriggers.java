package igknighters.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.LEDCommands.LEDSection;
import igknighters.commands.OperatorTarget.FaceSubLocation;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kLed;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.led.LedUtil.NamedLEDPattern;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import java.util.Set;
import monologue.Monologue;

public class SubsystemTriggers {

  @SuppressWarnings("unused")
  static int interpolateHeight(double elevatorHeight) {
    double min = SuperStructureState.AlgaeFloor.elevatorMeters - .1;
    double max = SuperStructureState.ScoreL4.elevatorMeters;
    double t = (elevatorHeight - min) / (max - min);
    Monologue.log("elevatorHeight", elevatorHeight);
    return Monologue.log("num LEDS", (int) (36.0 * (0.25 + (t * 0.75))));
  }

  static Color getTargetingColor() {
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      return new Color(0, 0, 255);
    } else {
      return new Color(255, 0, 0);
    }
  }

  @SuppressWarnings("unused")
  public static void setupTriggers(
      Subsystems subsystems, Localizer localizer, OperatorTarget target) {
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

    localizer
        .near(swerve.getYaw(), 1.0 * Conv.DEGREES_TO_RADIANS)
        .negate()
        .and(RobotModeTriggers.disabled())
        .onTrue(
            SwerveCommands.orientGyro(swerve, vision, localizer, localizer.pose().getRotation()))
        .onTrue(Commands.print("Reorienting robot to localizer pose"));
    final Trigger ledIdle = Triggers.subsystemIdle(led);

    final Trigger ledTriggerAutonomous =
        RobotModeTriggers.autonomous()
            .whileTrue(
                LEDCommands.runSplitWithLEDSection(
                    led,
                    new LEDSection(
                        0, 0, LedUtil.makeRainbow(255, 100), 36, "autonomous rainbow s1"),
                    new LEDSection(
                        1, 0, LedUtil.makeRainbow(255, 100), 36, "autonomous rainbow s2")));

    final Command ledDisabledLed =
        LEDCommands.runSplitWithLEDSection(
            led,
            new LEDSection(0, 0, LEDPattern.solid(Color.kRed), 36, "disabled red s1"),
            new LEDSection(1, 0, LEDPattern.solid(Color.kRed), 36, "disabled red s2"));
    Triggers.falseOnce().and(RobotModeTriggers.disabled()).whileTrue(ledDisabledLed);

    ledIdle
        .and(target.hasTarget())
        .onTrue(
            Commands.defer(
                    () ->
                        Commands.sequence(
                            LEDCommands.runSplitWithLEDSection(
                                    led,
                                    new LEDSection(
                                        1,
                                        0,
                                        new NamedLEDPattern(
                                            "blinkLeft",
                                            LedUtil.makeFlash(kLed.TargetingColor, .3)),
                                        interpolateHeight(
                                            target.superStructureState().elevatorMeters),
                                        "flashing color on left"),
                                    new LEDSection(
                                        0,
                                        0,
                                        LEDPattern.solid(kLed.TargetingColor),
                                        interpolateHeight(
                                            target.superStructureState().elevatorMeters),
                                        "solid color on right"))
                                .onlyIf(target.targeting(FaceSubLocation.LEFT)),
                            LEDCommands.runSplitWithLEDSection(
                                    led,
                                    new LEDSection(
                                        0,
                                        0,
                                        new NamedLEDPattern(
                                            "blinkIdk2", LedUtil.makeFlash(kLed.CoralColor, .3)),
                                        interpolateHeight(
                                            target.superStructureState().elevatorMeters),
                                        "flashy color center"),
                                    new LEDSection(
                                        1,
                                        0,
                                        new NamedLEDPattern(
                                            "blinkIdk1", LedUtil.makeFlash(kLed.CoralColor, .3)),
                                        interpolateHeight(
                                            target.superStructureState().elevatorMeters),
                                        "flashy color center"))
                                .onlyIf(target.targeting(FaceSubLocation.CENTER)),
                            LEDCommands.runSplitWithLEDSection(
                                    led,
                                    new LEDSection(
                                        0,
                                        0,
                                        new NamedLEDPattern(
                                            "blinkRight",
                                            LedUtil.makeFlash(kLed.TargetingColor, 1.0)),
                                        interpolateHeight(
                                            target.superStructureState().elevatorMeters),
                                        "flashing color on left"),
                                    new LEDSection(
                                        1,
                                        0,
                                        LEDPattern.solid(kLed.TargetingColor),
                                        interpolateHeight(
                                            target.superStructureState().elevatorMeters),
                                        "blue color on left"))
                                .onlyIf(target.targeting(FaceSubLocation.RIGHT))),
                    Set.of(led))
                .ignoringDisable(true));
  }
}
