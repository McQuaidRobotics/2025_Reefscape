package wayfinder.repulsorField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import wayfinder.repulsorField.Obstacle.HorizontalObstacle;
import wayfinder.repulsorField.Obstacle.VerticalObstacle;

public class FieldObstacles {
  public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
  public static final double FIELD_WIDTH = Units.inchesToMeters(317);

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

  public static final Obstacle[] REEF_VERTICES;

  static {
    REEF_VERTICES = new Obstacle[REEF_VERTICES_LOC.length];
    for (int i = 0; i < REEF_VERTICES_LOC.length; i++) {
      REEF_VERTICES[i] =
          new Obstacle.SnowmanObstacle(REEF_VERTICES_LOC[i], 3.0, .23, 0.3, 0.8, 0.4);
    }
  }

  public static Obstacle[] reefSides(int ignoreSide) {
    Obstacle[] sides = new Obstacle[REEF_VERTICES.length];
    for (int i = 0; i < REEF_VERTICES.length / 2; i++) {
      Translation2d blueInbetween = inBetween(REEF_VERTICES_LOC[i], REEF_VERTICES_LOC[(i + 1) % 6]);
      Translation2d redInbetween =
          inBetween(REEF_VERTICES_LOC[i + 6], REEF_VERTICES_LOC[(i + 1) % 6 + 6]);
      if (i == ignoreSide) {
        sides[i] = new Obstacle.PointObstacle(blueInbetween, 0.0, false);
        sides[i + 6] = new Obstacle.PointObstacle(redInbetween, 0.0, false);
      } else {
        sides[i] = new Obstacle.SnowmanObstacle(blueInbetween, 3.0, 1.0, 1.0, 0.0, 0.0);
        sides[i + 6] = new Obstacle.SnowmanObstacle(redInbetween, 3.0, 1.0, 1.0, 0.0, 0.0);
      }
    }
    return sides;
  }

  public static final Obstacle[] REEF_SMALL = {
    new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), 2.0, 1.45, 1.0, 4.0, 2.2),
    new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), 2.0, 1.45, 1.0, 4.0, 2.2)
  };

  public static final Obstacle[] REEF_LARGE = {
    new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), 2.8, 2.3, 1.3, 4.0, 2.2),
    new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), 2.8, 2.3, 1.3, 4.0, 2.2),
    new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), .1, 4.5, 1.5, 0.1, 5.2),
    new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), .1, 4.5, 1.5, 0.1, 5.2)
  };

  public static final Obstacle[] WALL =
      new Obstacle[] {
        new HorizontalObstacle(0.0, 0.5, .5, true),
        new HorizontalObstacle(FIELD_WIDTH, 0.5, .5, false),
        new VerticalObstacle(0.0, 0.5, .5, true),
        new VerticalObstacle(FIELD_LENGTH, 0.5, .5, false),
        new VerticalObstacle(7.55, 0.75, 0.5, false),
        new VerticalObstacle(10, 0.75, 0.5, true)
      };
}
