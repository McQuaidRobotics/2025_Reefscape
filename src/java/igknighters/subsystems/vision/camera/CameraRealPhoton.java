package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.AprilTags;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import igknighters.subsystems.vision.Vision.VisionUpdateFlaws;
import igknighters.util.logging.BootupLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An abstraction for a photon camera. */
public class CameraRealPhoton extends Camera {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Pose3d robotToCameraPoseOffset;
  private final PhotonPoseEstimator poseEstimator;
  private final Function<Double, Rotation2d> gyroYawSupplier;

  private Optional<VisionUpdate> previousUpdate = Optional.empty();
  private ArrayList<Integer> seenTags = new ArrayList<>();
  private ArrayList<VisionUpdate> updates = new ArrayList<>();

  public CameraRealPhoton(CameraConfig config, Function<Double, Rotation2d> gyroYawSupplier) {
    this.camera = new PhotonCamera(config.cameraName());
    this.robotToCamera = config.cameraTransform();
    this.gyroYawSupplier = gyroYawSupplier;
    this.robotToCameraPoseOffset = Pose3d.kZero.transformBy(robotToCamera);

    poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.APRIL_TAG_FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.robotToCamera);
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator.setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    BootupLogger.bootupLog("    " + config.cameraName() + " camera initialized (real)");
  }

  private Pose2d reproject(PhotonTrackedTarget target, Rotation2d gyroAngle) {
    Translation2d tagLoc = AprilTags.TAGS_POSE2D[target.fiducialId - 1].getTranslation();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Translation2d tagToRobotOffset =
        robotToCameraPoseOffset.transformBy(cameraToTag).toPose2d().getTranslation();
    tagToRobotOffset = tagToRobotOffset.rotateBy(gyroAngle);
    return new Pose2d(tagLoc.minus(tagToRobotOffset), gyroAngle);
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
    if (estRoboPose.targetsUsed.size() == 1) {
      var target = estRoboPose.targetsUsed.get(0);
      avgDistance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (DriverStation.isEnabled()) {
        // its assumed that the gyro is generally correct when enabled
        pose = reproject(target, gyroYawSupplier.apply(estRoboPose.timestampSeconds));
      }
    } else {
      avgDistance =
          estRoboPose.targetsUsed.stream()
              .map(PhotonTrackedTarget::getBestCameraToTarget)
              .map(Transform3d::getTranslation)
              .map(t3 -> Math.hypot(t3.getX(), t3.getY()))
              .mapToDouble(Double::doubleValue)
              .average()
              .orElseGet(() -> 100.0);
    }

    if (previousUpdate.isPresent()) {
      faults =
          VisionUpdateFlaws.solve(
              estRoboPose.estimatedPose,
              new Pose3d(previousUpdate.get().pose()),
              estRoboPose.timestampSeconds - previousUpdate.get().timestamp(),
              avgDistance,
              seenTags,
              List.of());
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
  public void periodic() {
    seenTags.clear();
    camera.getAllUnreadResults().stream()
        .filter(result -> result.hasTargets())
        .map(poseEstimator::update)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .map(this::update)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .forEach(updates::add);

    log("isConnected", camera.isConnected());
  }
}
