package sham.seasonspecific;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import java.util.Arrays;
import sham.ShamArena;
import wpilibExt.AllianceFlipper;

public class Reefscape {
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
                .map(pointAtBlue -> AllianceFlipper.flip(pointAtBlue))
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
