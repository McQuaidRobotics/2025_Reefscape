package igknighters.constants;

import static wayfinder.repulsorField.FieldObstacles.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.FieldConstants.Reef;
import wayfinder.repulsorField.Obstacle;

public class Pathing {
  private static final Rectangle2d FIELD =
      new Rectangle2d(
          FieldConstants.POSE2D_CENTER, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

  public enum PathObstacles {
    CLOSE_LEFT_REEF(Reef.CLOSE_LEFT_FACE, WALL, REEF_VERTICES, REEF_SMALL, reefSides(0)),
    CLOSE_MID_REEF(Reef.CLOSE_MID_FACE, WALL, REEF_VERTICES, REEF_SMALL, reefSides(1)),
    CLOSE_RIGHT_REEF(Reef.CLOSE_RIGHT_FACE, WALL, REEF_VERTICES, REEF_SMALL, reefSides(2)),
    FAR_LEFT_REEF(Reef.FAR_LEFT_FACE, WALL, REEF_VERTICES, REEF_SMALL, reefSides(3)),
    FAR_MID_REEF(Reef.FAR_MID_FACE, WALL, REEF_VERTICES, REEF_SMALL, reefSides(4)),
    FAR_RIGHT_REEF(Reef.FAR_RIGHT_FACE, WALL, REEF_VERTICES, REEF_SMALL, reefSides(5)),
    Other(WALL, REEF_LARGE);

    public final Rectangle2d hitBox;
    public final Obstacle[] obstacles;

    private static Rectangle2d faceHitBox(Pose2d face) {
      final Transform2d transform = new Transform2d(new Translation2d(-1.5, 0.0), Rotation2d.kZero);
      return new Rectangle2d(face.plus(transform), 3.0, 3.0);
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
