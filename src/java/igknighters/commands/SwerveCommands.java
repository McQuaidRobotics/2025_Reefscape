package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.SharedState;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.ControllerFactories;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.util.plumbing.TunableValues;
import monologue.GlobalField;
import wayfinder.controllers.Framework.ControllerSequence;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.TranslationController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.repulsorField.RepulsorFieldPlanner;
import wpilibExt.AllianceSymmetry;
import wpilibExt.Speeds;
import wpilibExt.Speeds.RobotSpeeds;

public class SwerveCommands {

  public static Trigger isSlowerThan(Swerve swerve, double speed) {
    return new Trigger(() -> swerve.getFieldSpeeds().magnitude() < speed);
  }

  public static Command commandStopDrives(final Swerve swerve) {
    return swerve.runOnce(() -> swerve.drive(RobotSpeeds.kZero)).withName("commandStopDrives");
  }

  public static Command orientGyro(Swerve swerve, Localizer localizer, Rotation2d orientation) {
    return swerve.runOnce(
        () -> {
          swerve.setYaw(orientation);
          var pose = new Pose2d(localizer.pose().getTranslation(), orientation);
          localizer.reset(pose);
        });
  }

  public static Command orientGyro(Swerve swerve, Localizer localizer) {
    return Commands.either(
        orientGyro(swerve, localizer, Rotation2d.kZero),
        orientGyro(swerve, localizer, Rotation2d.kPi),
        AllianceSymmetry::isBlue);
  }

  public static Command drive(Swerve swerve, final Speeds speeds) {
    return Commands.run(() -> swerve.drive(speeds), swerve);
  }

  public static Command driveVolts(Swerve swerve, Rotation2d robotRelativeRotation, double volts) {
    return Commands.run(() -> swerve.driveVolts(robotRelativeRotation, volts), swerve);
  }

  public static Command stop(Swerve swerve) {
    return Commands.runOnce(() -> swerve.drive(RobotSpeeds.kZero), swerve);
  }

  private static Command followRepulsor(
      RepulsorFieldPlanner planner,
      Swerve swerve,
      Localizer localizer,
      Pose2d target,
      ChassisConstraints constraints) {
    return swerve.startRun(
        () -> {
          planner.reset(localizer.pose(), swerve.getFieldSpeeds(), target);
        },
        () -> {
          if (TunableValues.getBoolean("ShowArrows", false).value()) {
            GlobalField.setObject("arrows", planner.getArrows(target.getTranslation(), 20, 10));
          }
          swerve.drive(
              planner.calculate(
                  ConstValues.PERIODIC_TIME,
                  localizer.pose(),
                  swerve.getFieldSpeeds(),
                  target,
                  constraints),
              constraints);
        });
  }

  public static Command lineupReef(
      Swerve swerve, Localizer localizer, Pose2d target, PathObstacles obstacles) {
    final ChassisConstraints preciseConstraints =
        new ChassisConstraints(
            new Constraints(kSwerve.MAX_DRIVE_VELOCITY * 0.3, kSwerve.MAX_DRIVE_ACCELERATION),
            new Constraints(
                kSwerve.MAX_ANGULAR_VELOCITY * 0.5, kSwerve.MAX_ANGULAR_VELOCITY * 0.65));
    final ChassisConstraints roughConstraints =
        new ChassisConstraints(
            new Constraints(
                kSwerve.MAX_DRIVE_VELOCITY * 0.8,
                SharedState.maximumAcceleration(SuperStructureState.Stow.elevatorMeters)),
            new Constraints(
                kSwerve.MAX_ANGULAR_VELOCITY * 1.25, kSwerve.MAX_ANGULAR_VELOCITY * 1.0));

    final RepulsorFieldPlanner precisePlanner =
        new RepulsorFieldPlanner(
            new PositionalController(
                new ControllerSequence<>(
                    TranslationController.profiled(2.25, 0.6, 0.0),
                    TranslationController.unprofiled(4.0, 0.0, 0.0, 0.025)),
                ControllerFactories.basicRotationalController()),
            obstacles.obstacles);
    final RepulsorFieldPlanner roughPlanner =
        new RepulsorFieldPlanner(
            new PositionalController(
                TranslationController.unprofiled(2.5, 0.0, 0.0, 0.0),
                RotationalController.unprofiled(4.5, 0.3, 0.0)),
            PathObstacles.Other.obstacles);

    final Transform2d roughPoseOffset = new Transform2d(-0.25, 0, Rotation2d.kZero);
    return Commands.sequence(
            followRepulsor(
                    roughPlanner, swerve, localizer, target.plus(roughPoseOffset), roughConstraints)
                .until(() -> obstacles.insideHitBox(localizer.pose().getTranslation())),
            followRepulsor(precisePlanner, swerve, localizer, target, preciseConstraints))
        .withName("lineupReef");
  }

  public static Command moveToSimple(Swerve swerve, Localizer localizer, Pose2d target) {
    final ChassisConstraints constraints =
        new ChassisConstraints(
            new Constraints(
                kSwerve.MAX_DRIVE_VELOCITY * 0.8,
                SharedState.maximumAcceleration(SuperStructureState.ScoreL4.elevatorMeters)),
            new Constraints(
                kSwerve.MAX_ANGULAR_VELOCITY * 0.5, kSwerve.MAX_ANGULAR_VELOCITY * 0.8));
    final PositionalController roughController =
        new PositionalController(
            new ControllerSequence<>(
              TranslationController.profiled(2.25, 0.6, 0.0),
              TranslationController.unprofiled(4.0, 0.0, 0.0, 0.025)
            ),
            ControllerFactories.basicRotationalController());
    return Commands.sequence(
            swerve.runOnce(
                () -> roughController.reset(localizer.pose(), swerve.getFieldSpeeds(), target)),
            swerve.run(
                () -> {
                  var speeds =
                      roughController.calculate(
                          ConstValues.PERIODIC_TIME,
                          localizer.pose(),
                          swerve.getFieldSpeeds(),
                          target,
                          constraints);
                  swerve.drive(speeds, constraints);
                }))
        .withName("moveToSimple");
  }
}
