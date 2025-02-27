package sham.seasonspecific;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import java.util.Arrays;
import java.util.List;
import org.dyn4j.geometry.Geometry;
import sham.ShamArena;
import sham.ShamGamePiece.GamePieceTarget;
import sham.ShamGamePiece.GamePieceVariant;
import wpilibExt.AllianceSymmetry;

public class Reefscape {
  public static final GamePieceVariant CORAL =
      new GamePieceVariant(
          "Coral",
          Units.inchesToMeters(4.5),
          0.7,
          Geometry.createRectangle(Units.inchesToMeters(4.5), Units.inchesToMeters(11.75)),
          List.of(),
          false,
          0);

  public static final GamePieceVariant ALGAE =
      new GamePieceVariant(
          "Algae",
          Units.inchesToMeters(16.0),
          0.5,
          Geometry.createCircle(Units.inchesToMeters(16.0)),
          List.of(
              new GamePieceTarget(
                  new Rectangle2d(new Pose2d(0.0, 0.0, Rotation2d.kZero), 0.5, 0.2),
                  new Pair<>(0.0, 1.0)),
              new GamePieceTarget(
                  new Rectangle2d(new Pose2d(0.0, 0.0, Rotation2d.kZero), 0.5, 0.2),
                  new Pair<>(0.0, 1.0))),
          true,
          0);

  public static class ReefscapeShamArena extends ShamArena {
    public static final class ReefscapeFieldObstacleMap extends FieldMap {
      public ReefscapeFieldObstacleMap() {
        super();

        // blue wall
        super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

        // blue coral stations
        super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672, 0));
        super.addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052));

        // red wall
        super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

        // red coral stations
        super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548 - 1.672, 0));
        super.addBorderLine(
            new Translation2d(17.548, 6.782), new Translation2d(17.548 - 1.672, 8.052));

        // upper walls
        super.addBorderLine(
            new Translation2d(1.672, 8.052), new Translation2d(17.548 - 1.672, 8.052));

        // lower walls
        super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(17.548 - 1.672, 0));

        // blue reef
        final Translation2d[] reefVorticesBlue =
            new Translation2d[] {
              new Translation2d(3.658, 3.546),
              new Translation2d(3.658, 4.506),
              new Translation2d(4.489, 4.987),
              new Translation2d(5.3213, 4.506),
              new Translation2d(5.3213, 3.546),
              new Translation2d(4.489, 3.065)
            };
        for (int i = 0; i < 6; i++) {
          super.addBorderLine(reefVorticesBlue[i], reefVorticesBlue[(i + 1) % 6]);
        }

        // red reef
        final Translation2d[] reefVorticesRed =
            Arrays.stream(reefVorticesBlue)
                .map(pointAtBlue -> AllianceSymmetry.flip(pointAtBlue))
                .toArray(Translation2d[]::new);
        for (int i = 0; i < 6; i++) {
          super.addBorderLine(reefVorticesRed[i], reefVorticesRed[(i + 1) % 6]);
        }

        // the pillar in the middle of the field
        super.addRectangularObstacle(0.305, 0.305, new Pose2d(8.774, 4.026, new Rotation2d()));
      }
    }

    public ReefscapeShamArena(Time period, int ticksPerPeriod) {
      super(new ReefscapeFieldObstacleMap(), period.in(Seconds), ticksPerPeriod);
    }

    @Override
    protected void competitionPeriodic() {}

    @Override
    protected void placeGamePiecesOnField() {}
  }
}
