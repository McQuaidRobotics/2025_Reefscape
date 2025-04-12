package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.SharedState;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.ControllerFactories;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.vision.Vision;
import igknighters.util.plumbing.TunableValues;
import monologue.GlobalField;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.TranslationController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.repulsorField.RepulsorFieldPlanner;
import wpilibExt.AllianceSymmetry;
import wpilibExt.Speeds;
import wpilibExt.Speeds.RobotSpeeds;

public class SwerveCommands {
  /**
   * Gets the angle between two points
   *
   * @param currentTrans The current translation
   * @param pose The pose to get the angle to
   * @param angleOffet An offset to add to the angle
   * @return The angle between the two points
   */
  public static Rotation2d rotationRelativeToPose(Translation2d currentTrans, Translation2d pose) {
    double angleBetween =
        Math.atan2(pose.getY() - currentTrans.getY(), pose.getX() - currentTrans.getX());
    return Rotation2d.fromRadians(angleBetween);
  }

  public static Command commandStopDrives(final Swerve swerve) {
    return swerve.runOnce(() -> swerve.drive(RobotSpeeds.kZero)).withName("commandStopDrives");
  }

  public static Command orientGyro(
      Swerve swerve, Vision vision, Localizer localizer, Rotation2d orientation) {
    return swerve.runOnce(
        () -> {
          vision.resetHeading();
          swerve.setYaw(orientation);
          var pose = new Pose2d(localizer.pose().getTranslation(), orientation);
          localizer.reset(pose);
        });
  }

  public static Command orientGyro(Swerve swerve, Vision vision, Localizer localizer) {
    return Commands.either(
        orientGyro(swerve, vision, localizer, Rotation2d.kZero),
        orientGyro(swerve, vision, localizer, Rotation2d.kPi),
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
                kSwerve.MAX_DRIVE_VELOCITY * 0.70,
                SharedState.maximumAcceleration(SuperStructureState.Stow.elevatorMeters)),
            new Constraints(
                kSwerve.MAX_ANGULAR_VELOCITY * 10.0, kSwerve.MAX_ANGULAR_VELOCITY * 10.0));

    final RepulsorFieldPlanner precisePlanner =
        new RepulsorFieldPlanner(
            new PositionalController(
                TranslationController.unprofiled(4.0, 0.0, 0.05, 0.01),
                ControllerFactories.basicRotationalController()),
            obstacles.obstacles);
    final RepulsorFieldPlanner roughPlanner =
        new RepulsorFieldPlanner(
            new PositionalController(
                TranslationController.unprofiled(2.25, 0.0, 0.0, 0.0),
                ControllerFactories.lowToleranceRotationalController()),
            PathObstacles.Other.obstacles);

    final Transform2d roughPoseOffset = new Transform2d(-0.3, 0, Rotation2d.kZero);
    return Commands.sequence(
            followRepulsor(
                    roughPlanner, swerve, localizer, target.plus(roughPoseOffset), roughConstraints)
                .until(() -> obstacles.insideHitBox(localizer.pose().getTranslation()))
                .unless(
                    () ->
                        localizer.pose().getTranslation().getDistance(target.getTranslation())
                            < 0.375),
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
            ControllerFactories.shortRangeTranslationController(),
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
