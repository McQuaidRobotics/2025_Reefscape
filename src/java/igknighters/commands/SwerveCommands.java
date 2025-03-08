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
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.vision.Vision;
import igknighters.util.plumbing.TunableValues;
import java.util.function.Supplier;
import monologue.GlobalField;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.TranslationController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.ControllerMode;
import wayfinder.repulsorField.RepulsorFieldPlanner;
import wpilibExt.AllianceFlipper;
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
        AllianceFlipper::isBlue);
  }

  public static Command drive(Swerve swerve, final Speeds speeds) {
    return Commands.run(() -> swerve.drive(speeds), swerve);
  }

  public static Command stop(Swerve swerve) {
    return Commands.runOnce(() -> swerve.drive(RobotSpeeds.kZero), swerve);
  }

  private static Command followRepulsor(
      RepulsorFieldPlanner planner,
      Swerve swerve,
      Localizer localizer,
      Pose2d target,
      Supplier<ChassisConstraints> constraints) {
    return swerve.run(
        () -> {
          if (TunableValues.getBoolean("ShowArrows", false).value()) {
            GlobalField.setObject("arrows", planner.getArrows(target.getTranslation(), 20, 10));
          }
          var c = constraints.get();
          swerve.drive(
              planner.calculate(
                  ConstValues.PERIODIC_TIME, localizer.pose(), swerve.getFieldSpeeds(), target, c),
              c);
        });
  }

  public static Command moveTo(
      Swerve swerve, Localizer localizer, Pose2d target, PathObstacles obstacles) {
    final ChassisConstraints constraints =
        new ChassisConstraints(
            new Constraints(
                kSwerve.MAX_DRIVE_VELOCITY * 0.5,
                SharedState.maximumAcceleration(SuperStructureState.ScoreL3.elevatorMeters)),
            new Constraints(
                kSwerve.MAX_ANGULAR_VELOCITY * 0.5, kSwerve.MAX_ANGULAR_VELOCITY * 0.8));
    final PositionalController preciseController =
        new PositionalController(
            new TranslationController(4.0, 0.00, 0.5, ControllerMode.UNPROFILED),
            new RotationalController(4.0, 0.2, ControllerMode.STRICT));
    final PositionalController roughController =
        new PositionalController(
            new TranslationController(
                kSwerve.MAX_DRIVE_VELOCITY, 0.0, 0.0, ControllerMode.UNPROFILED),
            new RotationalController(4.0, 0.2, ControllerMode.STRICT));
    final RepulsorFieldPlanner precisePlanner =
        new RepulsorFieldPlanner(preciseController, obstacles.obstacles);
    final RepulsorFieldPlanner roughPlanner =
        new RepulsorFieldPlanner(roughController, PathObstacles.Other.obstacles);

    final Transform2d roughPoseOffset = new Transform2d(1.0, 0, Rotation2d.kZero);
    return Commands.sequence(
        swerve.runOnce(
            () ->
                roughController.reset(
                    localizer.pose(), swerve.getFieldSpeeds(), target.plus(roughPoseOffset))),
        followRepulsor(roughPlanner, swerve, localizer, target, () -> constraints)
            .until(() -> obstacles.insideHitBox(localizer.pose().getTranslation())),
        swerve.runOnce(
            () -> preciseController.reset(localizer.pose(), swerve.getFieldSpeeds(), target)),
        followRepulsor(precisePlanner, swerve, localizer, target, () -> constraints));
  }

  public static Command moveToSimple(Swerve swerve, Localizer localizer, Pose2d target) {
    final ChassisConstraints constraints =
        new ChassisConstraints(
            new Constraints(
                kSwerve.MAX_DRIVE_VELOCITY * 0.8,
                SharedState.maximumAcceleration(SuperStructureState.ScoreL3.elevatorMeters)),
            new Constraints(
                kSwerve.MAX_ANGULAR_VELOCITY * 0.5, kSwerve.MAX_ANGULAR_VELOCITY * 0.8));
    final PositionalController roughController =
        new PositionalController(
            new TranslationController(2.0, 0.0, 0.0, ControllerMode.UNPROFILED),
            new RotationalController(2.0, 0.0, ControllerMode.UNPROFILED));
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
                      new Transform2d(),
                      constraints);
              swerve.drive(speeds, constraints);
            }));
  }
}
