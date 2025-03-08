package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import igknighters.constants.AprilTags;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import igknighters.subsystems.vision.Vision.VisionUpdateFlaws;
import igknighters.util.logging.BootupLogger;
import igknighters.util.plumbing.TunableValues;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An abstraction for a photon camera. */
public class CameraRealPhoton extends Camera {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;

  private Optional<VisionUpdate> previousUpdate = Optional.empty();
  private ArrayList<Integer> seenTags = new ArrayList<>();
  private ArrayList<VisionUpdate> updates = new ArrayList<>();

  public CameraRealPhoton(CameraConfig config, Function<Double, Rotation2d> gyroYawSupplier) {
    this.camera = new PhotonCamera(config.cameraName());
    this.robotToCamera = config.cameraTransform();

    poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.APRIL_TAG_FIELD, PoseStrategy.CONSTRAINED_SOLVEPNP, this.robotToCamera);
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

    BootupLogger.bootupLog("    " + config.cameraName() + " camera initialized (real)");
  }

  private Optional<VisionUpdate> update(EstimatedRobotPose estRoboPose) {
    for (PhotonTrackedTarget target : estRoboPose.targetsUsed) {
      int minId = AprilTags.TAGS[0].ID;
      int maxId = AprilTags.TAGS[AprilTags.TAGS.length - 1].ID;
      if (target.fiducialId < minId && target.fiducialId > maxId) {
        previousUpdate = Optional.empty();
        return previousUpdate;
      }
      seenTags.add(target.fiducialId);
    }

    VisionUpdateFlaws faults = VisionUpdateFlaws.empty();
    double avgDistance;
    Pose2d pose = estRoboPose.estimatedPose.toPose2d();
    log("tagsUsed", estRoboPose.targetsUsed.size());

    avgDistance =
        estRoboPose.targetsUsed.stream()
            .map(PhotonTrackedTarget::getBestCameraToTarget)
            .map(Transform3d::getTranslation)
            .map(t3 -> Math.hypot(t3.getX(), t3.getY()))
            .mapToDouble(Double::doubleValue)
            .average()
            .orElseGet(() -> 100.0);

    if (previousUpdate.isPresent()) {
      faults =
          VisionUpdateFlaws.solve(
              estRoboPose.estimatedPose,
              new Pose3d(previousUpdate.get().pose()),
              estRoboPose.timestampSeconds - previousUpdate.get().timestamp(),
              avgDistance,
              seenTags);
    }

    var u = new VisionUpdate(pose, estRoboPose.timestampSeconds, faults);
    previousUpdate = Optional.of(u);

    return previousUpdate;
  }

  @Override
  public Transform3d getRobotToCameraTransform3d() {
    return robotToCamera;
  }

  @Override
  public String getName() {
    return camera.getName();
  }

  @Override
  public List<VisionUpdate> flushUpdates() {
    var u = updates;
    updates = new ArrayList<>();
    return u;
  }

  @Override
  public List<Integer> getSeenTags() {
    return seenTags;
  }

  @Override
  public void updateHeading(double timestamp, Rotation2d heading) {
    poseEstimator.addHeadingData(timestamp, heading);
  }

  @Override
  public void periodic() {
    seenTags.clear();
    final var results = camera.getAllUnreadResults();
    PoseStrategy strategy =
        TunableValues.getBoolean("constrainedSolve", true).value()
            ? PoseStrategy.CONSTRAINED_SOLVEPNP
            : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;
    poseEstimator.setPrimaryStrategy(strategy);
    for (var result : results) {
      if (result.hasTargets()) {
        var estRoboPose =
            poseEstimator.update(
                result,
                camera.getCameraMatrix(),
                camera.getDistCoeffs(),
                Optional.of(
                    new ConstrainedSolvepnpParams(
                        false, TunableValues.getDouble("headingFactor", 0.0).value())));
        if (estRoboPose.isPresent()) {
          var u = update(estRoboPose.get());
          if (u.isPresent()) {
            updates.add(u.get());
          }
        }
      }
    }

    log("isConnected", camera.isConnected());
  }
}
