package igknighters.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.util.LerpTable;
import java.util.HashMap;

public class VisionConstants {
  public static final class kVision {
    public static final double ROOT_TRUST = 0.75;

    public static final double MAX_Z_DELTA = 0.2;
    public static final double MAX_ANGLE_DELTA = 5.0 * Conv.DEGREES_TO_RADIANS;

    public static final CameraConfig[] CONFIGS =
        new CameraConfig[] {
          new CameraConfig(
              "photon_front_forward_1",
              new Pose3d(
                  new Translation3d(
                      6.3 * Conv.INCHES_TO_METERS,
                      -11.5 * Conv.INCHES_TO_METERS,
                      7.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -17.5 * Conv.DEGREES_TO_RADIANS, 25.0 * Conv.DEGREES_TO_RADIANS))),
          new CameraConfig(
              "photon_front_forward_2",
              new Pose3d(
                  new Translation3d(
                      6.3 * Conv.INCHES_TO_METERS,
                      11.5 * Conv.INCHES_TO_METERS,
                      7.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -17.5 * Conv.DEGREES_TO_RADIANS, -25.0 * Conv.DEGREES_TO_RADIANS))),
          new CameraConfig(
              "photon_climber_1",
              new Pose3d(
                  new Translation3d(
                      kSwerve.MODULE_CHASSIS_LOCATIONS[1].getX(),
                      kSwerve.MODULE_CHASSIS_LOCATIONS[1].getY(),
                      7.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, -145.0 * Conv.DEGREES_TO_RADIANS))),
          new CameraConfig(
              "photon_climber_2",
              new Pose3d(
                  new Translation3d(
                      kSwerve.MODULE_CHASSIS_LOCATIONS[2].getX(),
                      kSwerve.MODULE_CHASSIS_LOCATIONS[2].getY(),
                      7.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, 145.0 * Conv.DEGREES_TO_RADIANS))),
        };

    public static final LerpTable DISTANCE_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 1.0),
            new LerpTable.LerpTableEntry(1.5, 1.0),
            new LerpTable.LerpTableEntry(2.5, 0.8),
            new LerpTable.LerpTableEntry(5.0, 0.5),
            new LerpTable.LerpTableEntry(8.0, 0.0));

    public static final HashMap<Integer, Double> TAG_RANKINGS =
        new HashMap<>() {
          {
            put(1, 1.0); // CORAL STATION
            put(2, 1.0); // CORAL STATION
            put(3, 1.0); // PROCESSOR
            put(4, 0.4); // BARGE
            put(5, 0.4); // BARGE
            put(6, 1.25); // REEF
            put(7, 1.25); // REEF
            put(8, 1.25); // REEF
            put(9, 1.25); // REEF
            put(10, 1.25); // REEF
            put(11, 1.25); // REEF
            put(12, 1.0); // CORAL STATION
            put(13, 1.0); // CORAL STATION
            put(14, 0.4); // BARGE
            put(15, 0.4); // BARGE
            put(16, 1.0); // PROCESSOR
            put(17, 1.25); // REEF
            put(18, 1.25); // REEF
            put(19, 1.25); // REEF
            put(20, 1.25); // REEF
            put(21, 1.25); // REEF
            put(22, 1.25); // REEF
          }
        };
  }
}
