package igknighters.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.Supplier;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.TranslationController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
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

  public static Command orientGyro(Swerve swerve, Localizer localizer) {
    return swerve.runOnce(
        () -> {
          if (AllianceFlipper.isBlue()) {
            swerve.setYaw(Rotation2d.kZero);
            var pose = new Pose2d(localizer.pose().getTranslation(), Rotation2d.kZero);
            localizer.reset(pose);
          } else {
            swerve.setYaw(Rotation2d.kPi);
            var pose = new Pose2d(localizer.pose().getTranslation(), Rotation2d.kPi);
            localizer.reset(pose);
          }
        });
  }

  // private abstract static class PointTowardsCommand extends Command {
  //   private final Swerve swerve;
  //   private final RotationalController rotController;
  //   private double velo = 0.0;

  //   public PointTowardsCommand(Swerve swerve) {
  //     this.swerve = swerve;
  //     this.rotController = new RotationalController(swerve);
  //     addRequirements(swerve);
  //   }

  //   abstract Rotation2d getTarget();

  //   @Override
  //   public void initialize() {
  //     rotController.reset();
  //   }

  //   @Override
  //   public void execute() {
  //     velo = rotController.calculate(getTarget().getRadians(), 0.0);
  //     swerve.drive(Speeds.fromFieldRelative(0.0, 0.0, velo));
  //   }

  //   @Override
  //   public boolean isFinished() {
  //     return Math.abs(velo) < 0.05;
  //   }

  //   @Override
  //   public String getName() {
  //     return "PointTowards";
  //   }
  // }

  // public static Command pointTowards(Swerve swerve, Translation2d target) {
  //   return new PointTowardsCommand(swerve) {
  //     @Override
  //     Rotation2d getTarget() {
  //       return rotationRelativeToPose(Translation2d.kZero, target);
  //     }
  //   };
  // }

  // public static Command pointTowards(Swerve swerve, Rotation2d target) {
  //   return new PointTowardsCommand(swerve) {
  //     @Override
  //     Rotation2d getTarget() {
  //       return target;
  //     }
  //   };
  // }

  public static Command drive(Swerve swerve, final Speeds speeds) {
    return Commands.run(() -> swerve.drive(speeds), swerve);
  }

  private static Command followRepulsor(
      RepulsorFieldPlanner planner,
      Swerve swerve,
      Localizer localizer,
      Pose2d target,
      Supplier<ChassisConstraints> constraints) {
    return swerve.run(
        () -> {
          // GlobalField.setObject("arrows", planner.getArrows(target.getTranslation(), 20, 10));
          var c = constraints.get();
          swerve.drive(
              planner.calculate(ConstValues.PERIODIC_TIME, localizer.pose(), target, c), c);
        });
  }

  public static Command moveTo(
      Swerve swerve, Localizer localizer, Pose2d target, PathObstacles obstacles) {
    final ChassisConstraints constraints =
        new ChassisConstraints(
            new Constraints(kSwerve.MAX_DRIVE_VELOCITY * 0.8, kSwerve.MAX_DRIVE_VELOCITY * 1.2),
            new Constraints(kSwerve.MAX_ANGULAR_VELOCITY, kSwerve.MAX_ANGULAR_VELOCITY * 0.8));
    final PositionalController controller =
        new PositionalController(
            new TranslationController(3.0, 0.13), new RotationalController(10.0, 0.2));
    final RepulsorFieldPlanner precisePlanner =
        new RepulsorFieldPlanner(controller, obstacles.obstacles);
    final RepulsorFieldPlanner roughPlanner =
        new RepulsorFieldPlanner(controller, PathObstacles.Other.obstacles);
    return Commands.sequence(
            swerve.runOnce(
                () -> controller.reset(localizer.pose(), swerve.getFieldSpeeds(), target)),
            followRepulsor(roughPlanner, swerve, localizer, target, () -> constraints)
                .until(() -> obstacles.hitBox.contains(localizer.pose().getTranslation())),
            followRepulsor(precisePlanner, swerve, localizer, target, () -> constraints))
        .until(() -> localizer.pose().getTranslation().getDistance(target.getTranslation()) < 0.05);
  }
}
