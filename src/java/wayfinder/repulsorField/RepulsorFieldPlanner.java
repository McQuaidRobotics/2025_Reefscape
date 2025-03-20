package wayfinder.repulsorField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import wayfinder.controllers.CircularSlewRateLimiter;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wayfinder.controllers.Types.ControllerSequence;
import wayfinder.controllers.Types.WrapperController;
import wpilibExt.MutTranslation2d;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Velocity2d;

public class RepulsorFieldPlanner {
  private final CircularSlewRateLimiter rotationRateLimiter =
      new CircularSlewRateLimiter(Math.PI * 5.0);
  private final Controller<Pose2d, FieldSpeeds, Pose2d, ChassisConstraints> controller;
  private final List<Obstacle> fixedObstacles = new ArrayList<>();

  private final MutTranslation2d netForceVec = new MutTranslation2d();

  @SuppressWarnings("unchecked")
  public RepulsorFieldPlanner(
      Controller<Translation2d, Velocity2d, Translation2d, Constraints> translationController,
      Controller<Rotation2d, Double, Rotation2d, Constraints> dynamicRotationController,
      Controller<Rotation2d, Double, Rotation2d, Constraints> staticRotationController,
      Obstacle... obstacles) {
    fixedObstacles.addAll(List.of(obstacles));
    var firstController =
        new PositionalController(translationController, dynamicRotationController);
    var secondController =
        new PositionalController(translationController, staticRotationController);
    controller =
        new ControllerSequence<>(
            new WrapperController<>(firstController) {
              public boolean isDone(Pose2d measurement, Pose2d target) {
                return measurement.getTranslation().getDistance(target.getTranslation()) < 0.375;
              }
            },
            secondController);
  }

  void getForce(Translation2d curLocation, Translation2d goal, MutTranslation2d out) {
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
      if (!Double.isFinite(xForceObs) || !Double.isFinite(yForceObs)) {
        continue;
      }
      xForceObs += force.getX();
      yForceObs += force.getY();
    }

    out.set(xForceGoal + xForceObs, yForceGoal + yForceObs);
  }

  Translation2d getForce(Translation2d curLocation, Translation2d goal) {
    MutTranslation2d out = new MutTranslation2d();
    getForce(curLocation, goal, out);
    return out;
  }

  public FieldSpeeds calculate(
      double period,
      Pose2d measurement,
      Speeds measurementVelo,
      Pose2d target,
      ChassisConstraints constraints) {
    double straightDist = measurement.getTranslation().getDistance(target.getTranslation());
    Pose2d intermediateTarget = target;
    if (straightDist > 0.375) {
      getForce(measurement.getTranslation(), target.getTranslation(), netForceVec);
      netForceVec.timesMut(straightDist / netForceVec.getNorm());
      Rotation2d forceDirection = netForceVec.getAngle();
      Rotation2d limited =
          new Rotation2d(rotationRateLimiter.calculate(forceDirection.getRadians()));
      Rotation2d targetDirection =
          target.getTranslation().minus(measurement.getTranslation()).getAngle();
      intermediateTarget =
          new Pose2d(
              measurement
                  .getTranslation()
                  .plus(netForceVec.rotateBy(limited.minus(forceDirection))),
              target.getRotation());
    }
    return controller.calculate(
        period,
        measurement,
        measurementVelo.asFieldRelative(measurement.getRotation()),
        intermediateTarget,
        constraints);
  }

  public void reset(Pose2d measurement, FieldSpeeds measurementVelo, Pose2d target) {
    controller.reset(measurement, measurementVelo, target);
    rotationRateLimiter.reset(
        measurement.getTranslation().minus(target.getTranslation()).getAngle().getRadians());
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
}
