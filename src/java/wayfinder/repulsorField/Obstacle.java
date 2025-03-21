package wayfinder.repulsorField;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wpilibExt.MutTranslation2d;

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
    private final Translation2d loc;
    private final double primaryMaxRange;
    private final double primaryRadius;
    private final double tailStrength;
    private final double tailDistance;

    // private final MutTranslation2d sidewaysPoint = new MutTranslation2d();
    // private final MutTranslation2d outwardsForce = new MutTranslation2d();
    // private final MutTranslation2d sidewaysForce = new MutTranslation2d();

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

    // public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
    //   final Rotation2d targetToLocAngle = loc.minus(target).getAngle();
    //   sidewaysPoint.setPolar(tailDistance, targetToLocAngle);
    //   sidewaysPoint.plusMut(loc);
    //   outwardsForce.setZero();
    //   sidewaysForce.setZero();

    //   final Translation2d positionToLocation = position.minus(loc);
    //   final double positionToLocationDistance = positionToLocation.getNorm();

    //   if (positionToLocationDistance <= primaryMaxRange) {
    //     final double magnitude =
    //         distToForceMag(
    //             Math.max(positionToLocationDistance - primaryRadius, 0),
    //             primaryMaxRange - primaryRadius);
    //     outwardsForce.setPolar(magnitude, positionToLocation.getAngle());
    //   }

    //   final Translation2d positionToLine =
    //       position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
    //   final double distanceAlongLine = positionToLine.getX();

    //   final double distanceScalar = distanceAlongLine / tailDistance;
    //   if (distanceScalar >= 0 && distanceScalar <= 1) {
    //     final double secondaryMaxRange =
    //         MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
    //     final double distanceToLine = Math.abs(positionToLine.getY());
    //     if (distanceToLine > secondaryMaxRange) {
    //       return outwardsForce;
    //     }
    //     final double sidewaysMag =
    //         tailStrength
    //             * (1 - distanceScalar * distanceScalar)
    //             * (secondaryMaxRange - distanceToLine);
    //     // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
    //     final Rotation2d targetToPositionAngle =
    //         new Rotation2d(target.getX() - position.getX(), target.getY() - position.getY());
    //     final Rotation2d positionToSidewaysPointUnaryAngle =
    //         new Rotation2d(
    //             position.getX() - sidewaysPoint.getX(), -(position.getY() -
    // sidewaysPoint.getY()));
    //     final double sidewaysThetaSin =
    //         (targetToPositionAngle.getCos() * positionToSidewaysPointUnaryAngle.getSin())
    //             + (targetToPositionAngle.getSin() * positionToSidewaysPointUnaryAngle.getCos());
    //     sidewaysForce.setPolar(sidewaysMag * Math.signum(sidewaysThetaSin), targetToLocAngle);
    //     sidewaysForce.rotateByMut(Rotation2d.kCCW_90deg);
    //   }

    //   outwardsForce.plusMut(sidewaysForce);
    //   return outwardsForce;
    // }

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

    final MutTranslation2d output = new MutTranslation2d();

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      output.set(Translation2d.kZero);
      var dist = Math.abs(position.getY() - y);
      if (dist < maxRange) {
        output.set(0, distToForceMag(y - position.getY(), maxRange));
      }
      return output;
    }
  }

  public static class VerticalObstacle extends Obstacle {
    final double x;
    final double maxRange;

    final MutTranslation2d output = new MutTranslation2d();

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      output.set(Translation2d.kZero);
      var dist = Math.abs(position.getX() - x);
      if (dist < maxRange) {
        output.set(distToForceMag(x - position.getX(), maxRange), 0);
      }
      return output;
    }
  }
}
