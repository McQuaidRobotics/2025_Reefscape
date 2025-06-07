package igknighters.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.subsystems.vision.camera.Camera.CameraIntrinsics;
import igknighters.util.LerpTable;
import java.util.HashMap;

public class VisionConstants {
  public static final class kVision {
    public static final double ROOT_TRUST = 1.0;

    public static final double MAX_Z_DELTA = 0.2;
    public static final double MAX_ANGLE_DELTA = 5.0 * Conv.DEGREES_TO_RADIANS;

    public static final CameraConfig[] CONFIGS =
        new CameraConfig[] {
          new CameraConfig(
              "front_left",
              new Pose3d(
                  new Translation3d(0.158, -0.269, 0.187),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, 25.0 * Conv.DEGREES_TO_RADIANS)),
              new CameraIntrinsics(
                  899.32,
                  899.04,
                  610.20,
                  386.27,
                  new double[] {0.037, -0.051, 0, -0.001, -0.004, -0.001, 0.002, 0})),
          new CameraConfig(
              "front_right",
              new Pose3d(
                  new Translation3d(0.203, 0.292, 0.176),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, -25.0 * Conv.DEGREES_TO_RADIANS)),
              new CameraIntrinsics(
                  907.53,
                  907.04,
                  679.9,
                  439.38,
                  new double[] {0.038, -0.031, 0, -0.001, -0.021, 0, 0.004, 0.003})),
          new CameraConfig(
              "back_left",
              new Pose3d(
                  new Translation3d(
                      -10.75 * Conv.INCHES_TO_METERS,
                      -10.75 * Conv.INCHES_TO_METERS,
                      8.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, -160.0 * Conv.DEGREES_TO_RADIANS)),
              new CameraIntrinsics(
                  909.81,
                  909.35,
                  652.63,
                  388.09,
                  new double[] {0.046, -0.071, -0.002, 0.001, 0.01, -0.002, 0.003, 0})),
          new CameraConfig(
              "back_right",
              new Pose3d(
                  new Translation3d(
                      -10.75 * Conv.INCHES_TO_METERS,
                      10.75 * Conv.INCHES_TO_METERS,
                      8.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, 160.0 * Conv.DEGREES_TO_RADIANS)),
              new CameraIntrinsics(
                  915.28,
                  914.69,
                  664.29,
                  387.37,
                  new double[] {0.028, -0.012, -0.001, 0.001, -0.032, 0, 0.002, 0.002})),
        };

    public static final LerpTable DISTANCE_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 1.0),
            new LerpTable.LerpTableEntry(0.65, 1.0),
            new LerpTable.LerpTableEntry(1.5, 0.7),
            new LerpTable.LerpTableEntry(2.5, 0.4),
            new LerpTable.LerpTableEntry(5.0, 0.25),
            new LerpTable.LerpTableEntry(8.0, 0.0));

    public static final LerpTable AREA_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 0.0),
            new LerpTable.LerpTableEntry(0.2, 0.45),
            new LerpTable.LerpTableEntry(1.0, 0.55),
            new LerpTable.LerpTableEntry(4.0, 0.75),
            new LerpTable.LerpTableEntry(7.5, 1.0));

    public static final LerpTable PIXEL_OFFSET_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 1.0),
            new LerpTable.LerpTableEntry(0.2, 1.0),
            new LerpTable.LerpTableEntry(0.65, 0.75),
            new LerpTable.LerpTableEntry(1.0, 0.35));

    public static final LerpTable LINEAR_VELOCITY_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 1.0),
            new LerpTable.LerpTableEntry(2.5, 0.8),
            new LerpTable.LerpTableEntry(kSwerve.MAX_DRIVE_VELOCITY, 0.1));

    public static final LerpTable ANGULAR_VELOCITY_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 1.0),
            new LerpTable.LerpTableEntry(7.0, 0.65),
            new LerpTable.LerpTableEntry(12.0, 0.0));

    public static final HashMap<Integer, Double> TAG_RANKINGS =
        new HashMap<>() {
          {
            put(1, 0.0); // CORAL STATION
            put(2, 0.0); // CORAL STATION
            put(3, 0.0); // PROCESSOR
            put(4, 0.0); // BARGE
            put(5, 0.0); // BARGE
            put(6, 1.0); // REEF
            put(7, 1.0); // REEF
            put(8, 1.0); // REEF
            put(9, 1.0); // REEF
            put(10, 1.0); // REEF
            put(11, 1.0); // REEF
            put(12, 0.0); // CORAL STATION
            put(13, 0.0); // CORAL STATION
            put(14, 0.0); // BARGE
            put(15, 0.0); // BARGE
            put(16, 0.0); // PROCESSOR
            put(17, 1.0); // REEF
            put(18, 1.0); // REEF
            put(19, 1.0); // REEF
            put(20, 1.0); // REEF
            put(21, 1.0); // REEF
            put(22, 1.0); // REEF
          }
        };

    public static final double MIN_TARGET_AREA = 0.25; // 0.25% of the image area
  }
}
