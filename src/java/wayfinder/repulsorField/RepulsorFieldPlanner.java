package wayfinder.repulsorField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import wayfinder.controllers.CircularSlewRateLimiter;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wpilibExt.Speeds.FieldSpeeds;

public class RepulsorFieldPlanner {
  private final PositionalController controller;
  private final CircularSlewRateLimiter angluarRateLimit =
      new CircularSlewRateLimiter(Math.PI * 15.0);
  private final List<Obstacle> fixedObstacles = new ArrayList<>();

  public RepulsorFieldPlanner(PositionalController controller, Obstacle... obstacles) {
    fixedObstacles.addAll(List.of(obstacles));
    this.controller = controller;
  }

  Translation2d getForce(Translation2d curLocation, Translation2d goal) {
    // push towards goal
    double xForceGoal = 0.0;
    double yForceGoal = 0.0;
    double xDisplacement = goal.getX() - curLocation.getX();
    double yDisplacement = goal.getY() - curLocation.getY();
    double norm = Math.hypot(xDisplacement, yDisplacement);
    if (norm != 0) {
      double cos = xDisplacement / norm;
      double sin = yDisplacement / norm;
      double mag = (1 + 1.0 / (1e-6 + norm));
      xForceGoal = mag * cos * 2.0;
      yForceGoal = mag * sin * 2.0;
    }

    // push away from obstacles
    double xForceObs = 0.0;
    double yForceObs = 0.0;
    for (Obstacle obs : fixedObstacles) {
      Translation2d force = obs.getForceAtPosition(curLocation, goal);
      xForceObs += force.getX();
      yForceObs += force.getY();

      if (!Double.isFinite(xForceObs) || !Double.isFinite(yForceObs)) {
        System.out.println("nan force");
      }
    }

    return new Translation2d(xForceGoal + xForceObs, yForceGoal + yForceObs);
  }

  public FieldSpeeds calculate(
      double period, Pose2d measurement, Pose2d target, ChassisConstraints constraints) {
    double straightDist = measurement.getTranslation().getDistance(target.getTranslation()) * 1.5;
    if (straightDist < 0.2) {
      return controller.calculate(period, measurement, target, Transform2d.kZero, constraints);
    } else {
      Translation2d netForce = getForce(measurement.getTranslation(), target.getTranslation());
      netForce = netForce.times(straightDist / netForce.getNorm());
      Rotation2d targetDirection = netForce.getAngle();
      Rotation2d limited = new Rotation2d(angluarRateLimit.calculate(targetDirection.getRadians()));
      netForce = netForce.rotateBy(limited.minus(targetDirection));
      return controller.calculate(
          period,
          measurement,
          new Pose2d(measurement.getTranslation().plus(netForce), target.getRotation()),
          Transform2d.kZero,
          constraints);
    }
  }

  public void reset(Pose2d measurement, FieldSpeeds measurementVelo, Pose2d target) {
    controller.reset(measurement, measurementVelo, target);
  }

  public Pose2d[] getArrows(Translation2d goal, double xCount, double yCount) {
    final double FIELD_WIDTH = 8.0518;
    final double FIELD_LENGTH = 17.54825;
    Pose2d[] arrows = new Pose2d[(int) (xCount * yCount + yCount + 1)];
    for (int x = 0; x <= xCount; x++) {
      for (int y = 0; y <= yCount; y++) {
        Translation2d translation =
            new Translation2d(x * (FIELD_LENGTH / 2.0) / xCount, y * FIELD_WIDTH / yCount);
        Translation2d force = getForce(translation, goal);
        Rotation2d rotation;
        if (force.getNorm() > 1e-6) {
          rotation = force.getAngle();
        } else {
          rotation = Rotation2d.kZero;
        }
        arrows[x * (int) yCount + y] = new Pose2d(translation, rotation);
      }
    }

    return arrows;
  }

  // public ArrayList<Translation2d> getTrajectory(
  //     Translation2d goal, Translation2d loc, double stepSize_m) {
  //   ArrayList<Translation2d> trajectory = new ArrayList<>();
  //   Translation2d robot = loc;
  //   for (int i = 0; i < 400; i++) {
  //     var err = robot.minus(goal);
  //     if (err.getNorm() < stepSize_m * 1.5) {
  //       trajectory.add(goal);
  //       break;
  //     } else {
  //       var netForce = getForce(robot, goal);
  //       if (netForce.getNorm() == 0) {
  //         break;
  //       }
  //       var step = new Translation2d(stepSize_m, netForce.getAngle());
  //       var intermediateGoal = robot.plus(step);
  //       trajectory.add(intermediateGoal);
  //       robot = intermediateGoal;
  //     }
  //   }
  //   return trajectory;
  // }
}
