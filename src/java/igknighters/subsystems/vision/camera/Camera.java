package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import igknighters.subsystems.Component;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import java.util.List;

public abstract class Camera extends Component {

  public record CameraIntrinsics(
      double width,
      double height,
      double fx,
      double fy,
      double cx,
      double cy,
      double[] distortion) {
    public CameraIntrinsics(double fx, double fy, double cx, double cy, double[] distortion) {
      this(1280, 800, fx, fy, cx, cy, distortion);
    }

    public Matrix<N8, N1> distortionMatrix() {
      return MatBuilder.fill(Nat.N8(), Nat.N1(), distortion);
    }

    public Matrix<N3, N3> cameraMatrix() {
      return MatBuilder.fill(Nat.N3(), Nat.N3(), new double[] {fx, 0, cx, 0, fy, cy, 0, 0, 1});
    }

    public double horizontalFOV() {
      return 2.0 * Math.atan2(width, 2.0 * fx);
    }

    public double verticalFOV() {
      return 2.0 * Math.atan2(height, 2.0 * fy);
    }

    public double diagonalFOV() {
      return 2.0 * Math.atan2(Math.hypot(width, height) / 2.0, fx);
    }
  }

  /**
   * A configuration for a camera. This allows to statically define cameras without instantiating
   * them.
   */
  public record CameraConfig(String cameraName, Pose3d cameraPose, CameraIntrinsics intrinsics) {
    public Transform3d cameraTransform() {
      return new Transform3d(Pose3d.kZero, cameraPose);
    }
  }

  /**
   * Uses the cameras PoseEstimation pipeline to estimate the pose of the robot.
   *
   * @return A list containing all updates since the last call to this method
   */
  public abstract List<VisionUpdate> flushUpdates();

  /**
   * Gets the transform from the robot to the camera.
   *
   * @return The transform
   * @apiNote This has to be very accurate, otherwise multi-camera pose estimation will suffer a
   *     lot.
   */
  public abstract Transform3d getRobotToCameraTransform3d();

  /**
   * Gets the name of the camera.
   *
   * @return The name of the camera
   */
  public abstract String getName();

  /**
   * Gets the last seen tags by the camera.
   *
   * @return The last seen tags
   */
  public abstract List<Integer> getSeenTags();

  /**
   * Updates any internal pose estimators to the robots heading.
   *
   * @param timestamp The timestamp of the update in FPGA time
   * @param heading The heading of the robot
   */
  public abstract void updateHeading(double timestamp, Rotation2d heading);

  /** Clears the heading buffer of the camera. */
  public abstract void clearHeading();
}
