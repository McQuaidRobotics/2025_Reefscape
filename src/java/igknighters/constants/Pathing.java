package igknighters.constants;

import static igknighters.constants.FieldConstants.FIELD_LENGTH;
import static igknighters.constants.FieldConstants.FIELD_WIDTH;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.FieldConstants.Reef;
import wayfinder.repulsorField.Obstacle;
import wayfinder.repulsorField.Obstacle.HorizontalObstacle;
import wayfinder.repulsorField.Obstacle.VerticalObstacle;

public class Pathing {

  private static final Translation2d[] REEF_VERTICES_LOC =
      new Translation2d[] {
        new Translation2d(3.658, 3.546),
        new Translation2d(3.658, 4.506),
        new Translation2d(4.489, 4.987),
        new Translation2d(5.3213, 4.506),
        new Translation2d(5.3213, 3.546),
        new Translation2d(FIELD_LENGTH - 4.489, 3.065),
        new Translation2d(FIELD_LENGTH - 3.658, 3.546),
        new Translation2d(FIELD_LENGTH - 3.658, 4.506),
        new Translation2d(FIELD_LENGTH - 4.489, 4.987),
        new Translation2d(FIELD_LENGTH - 5.3213, 4.506),
        new Translation2d(FIELD_LENGTH - 5.3213, 3.546),
        new Translation2d(FIELD_LENGTH - 4.489, 3.065)
      };

  private static Translation2d inBetween(Translation2d a, Translation2d b) {
    return a.interpolate(b, 0.5);
  }

  private static final Obstacle[] REEF_VERTICES;

  static {
    REEF_VERTICES = new Obstacle[REEF_VERTICES_LOC.length];
    for (int i = 0; i < REEF_VERTICES_LOC.length; i++) {
      REEF_VERTICES[i] =
          new Obstacle.SnowmanObstacle(REEF_VERTICES_LOC[i], 3.0, .23, 0.3, 0.8, 0.4);
    }
  }

  private static Obstacle[] reefSides(int ignoreSide) {
    Obstacle[] sides = new Obstacle[REEF_VERTICES.length];
    for (int i = 0; i < REEF_VERTICES.length / 2; i++) {
      Translation2d blueInbetween = inBetween(REEF_VERTICES_LOC[i], REEF_VERTICES_LOC[(i + 1) % 6]);
      Translation2d redInbetween =
          inBetween(REEF_VERTICES_LOC[i + 6], REEF_VERTICES_LOC[(i + 1) % 6 + 6]);
      if (i == ignoreSide) {
        sides[i] =
            new Obstacle(0.0, true) {
              @Override
              public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
                return Translation2d.kZero;
              }

              @Override
              protected double distToForceMag(double dist, double maxRange) {
                return 0.0;
              }
            };
        sides[i + 6] = sides[i];
      } else {
        sides[i] = new Obstacle.SnowmanObstacle(blueInbetween, 3.0, 1.0, 1.0, 0.05, 0.05);
        sides[i + 6] = new Obstacle.SnowmanObstacle(redInbetween, 3.0, 1.0, 1.0, 0.05, 0.05);
      }
    }
    return sides;
  }

  private static final Obstacle[] REEF_SMALL = {
    new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), 2.0, 1.45, 1.0, 4.0, 2.2),
    new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), 2.0, 1.45, 1.0, 4.0, 2.2)
  };

  private static final Obstacle[] REEF_LARGE = {
    new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), 1.0, 3.5, 1.2, 2.0, 2.2),
    new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), 1.0, 3.5, 1.2, 2.0, 2.2),
  };

  private static final Obstacle[] WALL =
      new Obstacle[] {
        new HorizontalObstacle(0.0, 0.3, .75, true),
        new HorizontalObstacle(FIELD_WIDTH, 0.3, .75, false),
        new VerticalObstacle(0.0, 0.3, .75, true),
        new VerticalObstacle(FIELD_LENGTH, 0.3, .75, false),
      };
  private static final Obstacle[] BARGE = {
    new VerticalObstacle(7.55, 0.3, 0.3, false), new VerticalObstacle(10, 0.3, 0.3, true)
  };

  private static final Rectangle2d FIELD =
      new Rectangle2d(
          FieldConstants.POSE2D_CENTER, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

  public enum PathObstacles {
    CLOSE_LEFT_REEF(
        Reef.Side.CLOSE_LEFT.face, WALL, BARGE, REEF_VERTICES, REEF_SMALL, reefSides(0)),
    CLOSE_MID_REEF(Reef.Side.CLOSE_MID.face, WALL, BARGE, REEF_VERTICES, REEF_SMALL, reefSides(1)),
    CLOSE_RIGHT_REEF(
        Reef.Side.CLOSE_RIGHT.face, WALL, BARGE, REEF_VERTICES, REEF_SMALL, reefSides(2)),
    FAR_LEFT_REEF(Reef.Side.FAR_LEFT.face, WALL, BARGE, REEF_VERTICES, REEF_SMALL, reefSides(3)),
    FAR_MID_REEF(Reef.Side.FAR_MID.face, WALL, BARGE, REEF_VERTICES, REEF_SMALL, reefSides(4)),
    FAR_RIGHT_REEF(Reef.Side.FAR_RIGHT.face, WALL, BARGE, REEF_VERTICES, REEF_SMALL, reefSides(5)),
    CAGE(WALL, REEF_LARGE),
    Other(WALL, REEF_LARGE);

    public final Rectangle2d hitBox;
    public final Obstacle[] obstacles;

    private static Rectangle2d faceHitBox(Pose2d face) {
      final Transform2d transform = new Transform2d(new Translation2d(1.0, 0.0), Rotation2d.kZero);
      return new Rectangle2d(face.plus(transform), 2.4, 2.25);
    }

    PathObstacles(Pose2d hitBox, Obstacle[]... obstacles) {
      this(faceHitBox(hitBox), obstacles);
    }

    PathObstacles(Obstacle[]... obstacles) {
      this(FIELD, obstacles);
    }

    PathObstacles(Rectangle2d hitBox, Obstacle[]... obstacles) {
      this.hitBox = hitBox;
      int totalLength = 0;
      for (var obstacleSet : obstacles) {
        totalLength += obstacleSet.length;
      }
      Obstacle[] all = new Obstacle[totalLength];
      int i = 0;
      for (var obstacleSet : obstacles) {
        for (var obstacle : obstacleSet) {
          all[i] = obstacle;
          i++;
        }
      }

      this.obstacles = all;
    }
  }
}
