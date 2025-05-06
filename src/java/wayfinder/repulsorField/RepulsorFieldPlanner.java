package wayfinder.repulsorField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import monologue.GlobalField;
import wayfinder.controllers.CircularSlewRateLimiter;
import wayfinder.controllers.Framework.Controller;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wpilibExt.MutTranslation2d;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;

public class RepulsorFieldPlanner
    implements Controller<Pose2d, Speeds, Pose2d, ChassisConstraints> {
  private final PositionalController controller;
  private final CircularSlewRateLimiter rotationRateLimiter =
      new CircularSlewRateLimiter(Math.PI * 5.0);
  private final List<Obstacle> fixedObstacles = new ArrayList<>();

  private final MutTranslation2d netForceVec = new MutTranslation2d();

  public RepulsorFieldPlanner(PositionalController controller, Obstacle... obstacles) {
    fixedObstacles.addAll(List.of(obstacles));
    this.controller = controller;
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

  @Override
  public FieldSpeeds calculate(
      double period,
      Pose2d measurement,
      Speeds measurementVelo,
      Pose2d target,
      ChassisConstraints constraints) {
    double straightDist = measurement.getTranslation().getDistance(target.getTranslation());
    Pose2d intermediatePose = target;
    if (straightDist > 0.375) {
      getForce(measurement.getTranslation(), target.getTranslation(), netForceVec);
      netForceVec.timesMut(straightDist / netForceVec.getNorm());
      Rotation2d targetDirection = netForceVec.getAngle();
      Rotation2d limited =
          new Rotation2d(rotationRateLimiter.calculate(targetDirection.getRadians()));
      intermediatePose =
          new Pose2d(
              measurement
                  .getTranslation()
                  .plus(netForceVec.rotateBy(limited.minus(targetDirection))),
              target.getRotation());
    }
    GlobalField.setObject("intermediatePose", intermediatePose);
    return controller.calculate(
        period, measurement, measurementVelo, intermediatePose, constraints);
  }

  @Override
  public void reset(Pose2d measurement, Speeds measurementVelo, Pose2d target) {
    controller.reset(measurement, measurementVelo, target);
    rotationRateLimiter.reset(
        target.getTranslation().minus(measurement.getTranslation()).getAngle().getRadians());
  }

  @Override
  public boolean isDone(Pose2d measurement, Pose2d target) {
    return controller.isDone(measurement, target);
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
