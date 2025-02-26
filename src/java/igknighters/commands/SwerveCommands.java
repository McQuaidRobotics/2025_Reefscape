package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.constants.FieldConstants.Reef;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.util.plumbing.TunableValues;
import java.util.function.Supplier;
import monologue.GlobalField;
import monologue.Monologue;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.TranslationController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.repulsorField.RepulsorFieldPlanner;
import wpilibExt.AllianceFlipper;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;
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

  public static Command moveToReef(
      Swerve swerve, Localizer localizer, Pose2d target, PathObstacles obstacles, double endDist) {
    final ChassisConstraints constraints =
        new ChassisConstraints(
            new Constraints(kSwerve.MAX_DRIVE_VELOCITY * 0.8, kSwerve.MAX_DRIVE_VELOCITY * 1.2),
            new Constraints(kSwerve.MAX_ANGULAR_VELOCITY, kSwerve.MAX_ANGULAR_VELOCITY * 0.8));

    final RotationalController rotController = new RotationalController(10.0, 0.2, true);
    final PositionalController controller =
        new PositionalController(
            new TranslationController(3.0, 0.09, 0.19, false),
            new RotationalController(10.0, 0.2, false));

    final RepulsorFieldPlanner precisePlanner =
        new RepulsorFieldPlanner(controller, obstacles.obstacles);
    final RepulsorFieldPlanner roughPlanner =
        new RepulsorFieldPlanner(controller, PathObstacles.Other.obstacles);

    final Command moveRough =
        swerve.run(
            () -> {
              var c = constraints;
              FieldSpeeds measuredSpeeds = swerve.getFieldSpeeds();
              FieldSpeeds speeds =
                  roughPlanner.calculate(
                      ConstValues.PERIODIC_TIME, localizer.pose(), measuredSpeeds, target, c);
              Translation2d reefCenter =
                  AllianceFlipper.isBlue() ? Reef.CENTER : AllianceFlipper.flip(Reef.CENTER);
              double rotSpeeds =
                  rotController.calculate(
                      ConstValues.PERIODIC_TIME,
                      localizer.pose().getRotation().getRadians(),
                      measuredSpeeds.omega(),
                      Monologue.log(
                          "targetAngle",
                          localizer
                              .pose()
                              .getTranslation()
                              .minus(reefCenter)
                              .getAngle()
                              .rotateBy(Rotation2d.k180deg)
                              .getRadians()),
                      Units.degreesToRadians(1.0),
                      c.rotation());
              FieldSpeeds newSpeeds = new FieldSpeeds(speeds.vx(), speeds.vy(), rotSpeeds);
              swerve.drive(newSpeeds, c);
            });

    return Commands.sequence(
            swerve.runOnce(
                () -> {
                  roughPlanner.reset(localizer.pose(), swerve.getFieldSpeeds(), target);
                  rotController.reset(
                      localizer.pose().getRotation().getRadians(), swerve.getFieldSpeeds().omega());
                }),
            moveRough.until(() -> obstacles.insideHitBox(localizer.pose().getTranslation())),
            swerve.runOnce(
                () -> precisePlanner.reset(localizer.pose(), swerve.getFieldSpeeds(), target)),
            followRepulsor(precisePlanner, swerve, localizer, target, () -> constraints))
        .until(
            () -> localizer.pose().getTranslation().getDistance(target.getTranslation()) < endDist);
  }
}
