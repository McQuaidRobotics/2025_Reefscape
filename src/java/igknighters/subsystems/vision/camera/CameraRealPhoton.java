package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import igknighters.constants.AprilTags;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import igknighters.subsystems.vision.VisionConstants.kVision;
import igknighters.util.logging.BootupLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An abstraction for a photon camera. */
public class CameraRealPhoton extends Camera {
  private static final Optional<ConstrainedSolvepnpParams> CONSTRAINED_PARAMS =
      Optional.of(new ConstrainedSolvepnpParams(true, 0.0));

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final double trustScalar;

  private final Optional<Matrix<N3, N3>> cameraMatrix;
  private final Optional<Matrix<N8, N1>> distortionMatrix;

  private Optional<VisionUpdate> previousUpdate = Optional.empty();
  private ArrayList<Integer> seenTags = new ArrayList<>();
  private ArrayList<VisionUpdate> updates = new ArrayList<>();

  public CameraRealPhoton(CameraConfig config) {
    this.camera = new PhotonCamera(config.cameraName());
    this.cameraMatrix = Optional.of(config.intrinsics().cameraMatrix());
    this.distortionMatrix = Optional.of(config.intrinsics().distortionMatrix());
    this.robotToCamera = config.cameraTransform();

    poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.APRIL_TAG_FIELD, PoseStrategy.CONSTRAINED_SOLVEPNP, this.robotToCamera);
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (config.cameraName().contains("back")) {
      trustScalar = 0.65;
    } else {
      trustScalar = 1.0;
    }

    BootupLogger.bootupLog("    " + config.cameraName() + " camera initialized (real)");
  }

  private double normalizedDistanceFromCenter(PhotonTrackedTarget target) {
    final double HEIGHT = 800;
    final double WIDTH = 1280;
    double sumX = 0.0;
    double sumY = 0.0;
    for (var corner : target.minAreaRectCorners) {
      sumX += corner.x - WIDTH / 2.0;
      sumY += corner.y - HEIGHT / 2.0;
    }
    double avgX = sumX / target.minAreaRectCorners.size();
    double avgY = sumY / target.minAreaRectCorners.size();
    return Math.hypot(avgX, avgY) / Math.hypot(WIDTH / 2.0, HEIGHT / 2.0);
  }

  private Optional<VisionUpdate> update(EstimatedRobotPose estRoboPose) {
    for (PhotonTrackedTarget target : estRoboPose.targetsUsed) {
      seenTags.add(target.fiducialId);
    }

    log("tagsUsed", estRoboPose.targetsUsed.size());

    double trust = trustScalar;

    Pose2d pose = estRoboPose.estimatedPose.toPose2d();

    double avgDistance =
        estRoboPose.targetsUsed.stream()
            .map(PhotonTrackedTarget::getBestCameraToTarget)
            .map(Transform3d::getTranslation)
            .map(t3 -> Math.hypot(t3.getX(), t3.getY()))
            .mapToDouble(Double::doubleValue)
            .average()
            .orElseGet(() -> 100.0);

    double sumArea =
        estRoboPose.targetsUsed.stream()
            .map(PhotonTrackedTarget::getArea)
            .mapToDouble(Double::doubleValue)
            .sum();

    double avgNormalizedPixelsFromCenter =
        estRoboPose.targetsUsed.stream()
            .map(this::normalizedDistanceFromCenter)
            .mapToDouble(Double::doubleValue)
            .average()
            .orElseGet(() -> 0.0);

    if (previousUpdate.isPresent()) {
      double timeSinceLastUpdate = estRoboPose.timestampSeconds - previousUpdate.get().timestamp();
      double distanceFromLastUpdate =
          pose.getTranslation().getDistance(previousUpdate.get().pose().getTranslation());
      if (distanceFromLastUpdate > timeSinceLastUpdate * kSwerve.MAX_DRIVE_VELOCITY) {
        return Optional.empty();
      }
    }

    for (int tagId : seenTags) {
      trust *= kVision.TAG_RANKINGS.getOrDefault(tagId, 0.0);
    }

    trust *= kVision.DISTANCE_TRUST_COEFFICIENT.lerp(avgDistance);
    trust *= kVision.AREA_TRUST_COEFFICIENT.lerp(sumArea);
    trust *= kVision.PIXEL_OFFSET_TRUST_COEFFICIENT.lerp(avgNormalizedPixelsFromCenter);

    if (DriverStation.isDisabled()) {
      trust = 1.0;
    }

    var u = new VisionUpdate(pose, estRoboPose.timestampSeconds, trust);
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

  private PhotonPipelineResult pruneTags(PhotonPipelineResult result) {
    ArrayList<PhotonTrackedTarget> newTargets = new ArrayList<>();
    for (var target : result.targets) {
      if (AprilTags.observalbleTag(target.fiducialId)) {
        newTargets.add(target);
      }
    }
    result.targets = newTargets;
    return result;
  }

  @Override
  public void periodic() {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), Rotation2d.kZero);
    seenTags.clear();
    final var results = camera.getAllUnreadResults();
    for (var result : results) {
      if (result.hasTargets()) {
        result = pruneTags(result);
        Optional<EstimatedRobotPose> estRoboPose = poseEstimator.update(result, cameraMatrix, distortionMatrix, CONSTRAINED_PARAMS);
        if (estRoboPose.isPresent()) {
          Optional<VisionUpdate> u = update(estRoboPose.get());
          if (u.isPresent()) {
            updates.add(u.get());
          }
        }
      }
    }

    log("isConnected", camera.isConnected());
  }
}
