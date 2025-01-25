package wayfinder.repulsorField;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class Obstacle {
  double strength;
  boolean positive;

  public Obstacle(double strength, boolean positive) {
    this.strength = strength;
    this.positive = positive;
  }

  public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d goal);

  protected double distToForceMag(double dist, double maxRange) {
    if (MathUtil.isNear(0, dist, 1e-2)) {
      dist = 1e-2;
    }
    var forceMag = strength / (dist * dist);
    forceMag -= strength / (maxRange * maxRange);
    forceMag *= positive ? 1 : -1;
    return forceMag;
  }

  protected double rotateBy(double radians, double rhs) {
    return Math.cos(radians) * rhs;
  }

  public static class PointObstacle extends Obstacle {
    Translation2d loc;
    double effectMaxRange = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > effectMaxRange) {
        return Translation2d.kZero;
      }
      var outwardsMag = distToForceMag(loc.getDistance(position), effectMaxRange);
      var outwardsVector = new Translation2d(outwardsMag, position.minus(loc).getAngle());
      // theta = angle between position->target vector and obstacle->position vector
      var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      var sidewaysVector =
          outwardsVector.rotateBy(Rotation2d.kCCW_90deg).div(outwardsVector.getNorm()).times(mag);

      return outwardsVector.plus(sidewaysVector);
    }
  }

  public static class SnowmanObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double secondaryDistance;
    final double secondaryMaxRange;
    final double secondaryStrengthRatio;

    public SnowmanObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double secondaryDistance,
        double secondaryStrength,
        double secondaryMaxRange) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.secondaryDistance = secondaryDistance;
      this.secondaryMaxRange = secondaryMaxRange;
      secondaryStrengthRatio = primaryStrength / secondaryStrength;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysCircle = new Translation2d(secondaryDistance, targetToLoc.getAngle()).plus(loc);
      var dist = loc.getDistance(position);
      var sidewaysDist = sidewaysCircle.getDistance(position);
      if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
        return Translation2d.kZero;
      }
      var sidewaysMag =
          distToForceMag(sidewaysCircle.getDistance(position), primaryMaxRange)
              / secondaryStrengthRatio;
      var outwardsMag = distToForceMag(loc.getDistance(position), secondaryMaxRange);
      var initial = new Translation2d(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      var sidewaysTheta =
          target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
      return new Translation2d(sideways, sidewaysAngle).plus(initial);
    }
  }

  public static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailDistance;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailDistance = tailLength + primaryMaxRange;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysPoint = new Translation2d(tailDistance, targetToLoc.getAngle()).plus(loc);

      var positionToLocation = position.minus(loc);
      var positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce =
            new Translation2d(
                distToForceMag(
                    Math.max(positionToLocationDistance - primaryRadius, 0),
                    primaryMaxRange - primaryRadius),
                positionToLocation.getAngle());
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      var distanceAlongLine = positionToLine.getX();

      Translation2d sidewaysForce;
      var distanceScalar = distanceAlongLine / tailDistance;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        var secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        var distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          var sidewaysMag =
              tailStrength
                  * (1 - distanceScalar * distanceScalar)
                  * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
          var sidewaysTheta =
              target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce =
              new Translation2d(
                  sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                  targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      return outwardsForce.plus(sidewaysForce);
    }
  }

  public static class HorizontalObstacle extends Obstacle {
    final double y;
    final double maxRange;

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getY() - y);
      if (dist > maxRange) {
        return Translation2d.kZero;
      }
      return new Translation2d(0, distToForceMag(y - position.getY(), maxRange));
    }
  }

  public static class VerticalObstacle extends Obstacle {
    final double x;
    final double maxRange;

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getX() - x);
      if (dist > maxRange) {
        return Translation2d.kZero;
      }
      return new Translation2d(distToForceMag(x - position.getX(), maxRange), 0);
    }
  }
}
