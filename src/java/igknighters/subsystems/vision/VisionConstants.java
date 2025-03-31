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
    public static final double ROOT_TRUST = 0.75;

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
              // TODO
              new CameraIntrinsics(
                  913.51,
                  912.47,
                  660.17,
                  397.42,
                  new double[] {0.049, -0.08, 0.0, 0.0, 0.024, -0.002, 0.004, 0.0})),
          new CameraConfig(
              "front_right",
              new Pose3d(
                  new Translation3d(0.203, 0.292, 0.176),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, -25.0 * Conv.DEGREES_TO_RADIANS)),
              new CameraIntrinsics(
                  913.61,
                  912.90,
                  641.63,
                  363.79,
                  new double[] {0.046, -0.066, 0.0, 0.0, 0.013, -0.002, 0.005, 0.001})),
          new CameraConfig(
              "back_left",
              new Pose3d(
                  new Translation3d(
                      -10.75 * Conv.INCHES_TO_METERS,
                      -10.75 * Conv.INCHES_TO_METERS,
                      8.0 * Conv.INCHES_TO_METERS),
                  new Rotation3d(
                      0.0, -15.0 * Conv.DEGREES_TO_RADIANS, -160.0 * Conv.DEGREES_TO_RADIANS)),
              // TODO
              new CameraIntrinsics(
                  913.61,
                  912.90,
                  641.63,
                  363.79,
                  new double[] {0.046, -0.066, 0.0, 0.0, 0.013, -0.002, 0.005, 0.001})),
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
                  913.51,
                  912.47,
                  660.17,
                  397.42,
                  new double[] {0.049, -0.08, 0.0, 0.0, 0.024, -0.002, 0.004, 0.0})),
        };

    public static final LerpTable DISTANCE_TRUST_COEFFICIENT =
        new LerpTable(
            new LerpTable.LerpTableEntry(0.0, 1.0),
            new LerpTable.LerpTableEntry(1.5, 1.0),
            new LerpTable.LerpTableEntry(2.5, 0.8),
            new LerpTable.LerpTableEntry(5.0, 0.5),
            new LerpTable.LerpTableEntry(8.0, 0.0));

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
  }
}
