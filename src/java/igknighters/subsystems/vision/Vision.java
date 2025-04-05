package igknighters.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.SharedState;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import igknighters.subsystems.vision.VisionConstants.kVision;
import igknighters.subsystems.vision.camera.Camera;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.subsystems.vision.camera.CameraDisabled;
import igknighters.subsystems.vision.camera.CameraRealPhoton;
import igknighters.subsystems.vision.camera.CameraSimPhoton;
import igknighters.util.plumbing.Channel.Receiver;
import igknighters.util.plumbing.Channel.Sender;
import igknighters.util.plumbing.TunableValues;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import monologue.Annotations.IgnoreLogged;
import monologue.GlobalField;
import monologue.ProceduralStructGenerator;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Tracer;

public class Vision implements SharedSubsystem {
  private final SharedState shared;
  @IgnoreLogged private final Localizer localizer;

  private final Sender<VisionSample> visionSender;
  private final Receiver<SwerveDriveSample> swerveDataReceiver;

  private final Camera[] cameras;

  private final Timer timerSinceLastSample = new Timer();
  private final HashSet<Integer> seenTags = new HashSet<>();

  public record VisionUpdateFlaws(
      boolean extremeJitter, boolean infeasiblePosition, double avgDistance, double tagMult)
      implements StructSerializable {

    private static final VisionUpdateFlaws kEmpty = new VisionUpdateFlaws(false, false, 0.0, 1.0);

    public static VisionUpdateFlaws empty() {
      return kEmpty;
    }

    public static VisionUpdateFlaws solve(
        Pose3d pose,
        Pose3d lastPose,
        double time,
        double avgDistance,
        List<Integer> tagsList,
        double trustScalar) {
      Translation2d simplePose = pose.getTranslation().toTranslation2d();
      boolean outOfBounds =
          simplePose.getX() < 0.0
              || simplePose.getX() > FieldConstants.FIELD_LENGTH
              || simplePose.getY() < 0.0
              || simplePose.getY() > FieldConstants.FIELD_WIDTH
              || Double.isNaN(simplePose.getX())
              || Double.isNaN(simplePose.getY());
      boolean extremeJitter =
          pose.getTranslation().getDistance(lastPose.getTranslation())
              > time * kSwerve.MAX_DRIVE_VELOCITY;
      boolean infeasibleZValue = Math.abs(pose.getTranslation().getZ()) > kVision.MAX_Z_DELTA;
      boolean infeasiblePitchValue = pose.getRotation().getY() > kVision.MAX_ANGLE_DELTA;
      boolean infeasibleRollValue = pose.getRotation().getX() > kVision.MAX_ANGLE_DELTA;
      double tagTrust = 1.0;
      for (int tagId : tagsList) {
        tagTrust *= kVision.TAG_RANKINGS.getOrDefault(tagId, 1.0);
      }
      return new VisionUpdateFlaws(
          extremeJitter,
          infeasiblePitchValue || infeasibleRollValue || infeasibleZValue || outOfBounds,
          avgDistance,
          tagTrust * trustScalar);
    }

    public static final Struct<VisionUpdateFlaws> struct =
        ProceduralStructGenerator.genRecord(VisionUpdateFlaws.class);
  }

  public record VisionUpdate(Pose2d pose, double timestamp, VisionUpdateFlaws flaws)
      implements StructSerializable {

    private static final VisionUpdate kEmpty =
        new VisionUpdate(Pose2d.kZero, 0.0, VisionUpdateFlaws.empty());

    public static VisionUpdate empty() {
      return kEmpty;
    }

    public static final Struct<VisionUpdate> struct =
        ProceduralStructGenerator.genRecord(VisionUpdate.class);
  }

  public record VisionSample(Pose2d pose, double timestamp, double trust)
      implements StructSerializable {

    public static final Struct<VisionSample> struct =
        ProceduralStructGenerator.genRecord(VisionSample.class);
  }

  private Rotation2d rotationAtTimestamp(double timestamp) {
    return localizer
        .pose(Timer.getFPGATimestamp() - timestamp)
        .map(Pose2d::getRotation)
        .orElse(null);
  }

  private Camera makeCamera(CameraConfig config, SimCtx simCtx) {
    try {
      if (Robot.isSimulation()) {
        return new CameraSimPhoton(config, simCtx, this::rotationAtTimestamp);
        // return new CameraDisabled(config.cameraName(), config.cameraTransform());
      } else {
        return new CameraRealPhoton(config, this::rotationAtTimestamp);
        // return new CameraDisabled(config.cameraName(), config.cameraTransform());
      }
    } catch (Exception e) {
      // if the camera fails to initialize, return a disabled camera to not crash the code
      DriverStation.reportError("Failed to initialize camera: " + config.cameraName(), false);
      return new CameraDisabled(config.cameraName(), config.cameraTransform());
    }
  }

  public Vision(SharedState shared, final Localizer localizer, final SimCtx simCtx) {
    this.shared = shared;
    this.localizer = localizer;
    final var configs = kVision.CONFIGS;
    this.cameras = new Camera[configs.length];
    for (int i = 0; i < configs.length; i++) {
      this.cameras[i] = makeCamera(configs[i], simCtx);
    }

    visionSender = localizer.visionDataSender();
    swerveDataReceiver = localizer.swerveDataReceiver();
  }

  private Optional<VisionSample> gaugeTrust(final VisionUpdate update) {
    final VisionUpdateFlaws faults = update.flaws();

    if (faults.extremeJitter) {
      return Optional.empty();
    }

    double trust = kVision.ROOT_TRUST * TunableValues.getDouble("visionTrustScaler", 1.0).value();

    // If the average distance of the tags is too far away reduce the trust
    trust *= kVision.DISTANCE_TRUST_COEFFICIENT.lerp(update.flaws.avgDistance);

    // If there is a sketchy tag reduce the trust
    trust *= update.flaws.tagMult;

    // Completely arbitrary values for the velocity thresholds.
    // When the robot is moving fast there can be paralaxing and motion blur
    // that can cause the vision system to be less accurate, reduce the trust due to this
    FieldSpeeds velo = shared.fieldSpeeds;
    trust *= kVision.LINEAR_VELOCITY_TRUST_COEFFICIENT.lerp(velo.magnitude());
    trust *= kVision.ANGULAR_VELOCITY_TRUST_COEFFICIENT.lerp(velo.omega());

    // If the vision rotation varies significantly from the gyro rotation reduce the trust
    // Rotation2d rotation = localizer.pose().getRotation();
    // if (Math.abs(
    //         MathUtil.angleModulus(rotation.getRadians())
    //             - MathUtil.angleModulus(update.pose().getRotation().getRadians()))
    //     > Math.toRadians(5.0)) {
    //   trust /= 2.0;
    // }

    // if (faults.infeasiblePosition) {
    //   trust /= 3.0;
    // }

    return Optional.of(new VisionSample(update.pose(), update.timestamp(), trust));
  }

  public void resetHeading() {
    for (final Camera camera : cameras) {
      camera.clearHeading();
    }
  }

  public double timeSinceLastSample() {
    return timerSinceLastSample.get();
  }

  @Override
  public void periodic() {
    Tracer.startTrace("VisionPeriodic");

    final SwerveDriveSample[] swerveSamples = swerveDataReceiver.recvAll();

    for (final Camera camera : cameras) {
      Tracer.startTrace(camera.getName() + "Periodic");

      if (DriverStation.isEnabled()) {
        for (final SwerveDriveSample sample : swerveSamples) {
          camera.updateHeading(sample.timestamp(), sample.gyroYaw());
        }
      }

      try {
        camera.periodic();
      } catch (Exception e) {
        DriverStation.reportError("Error in camera " + camera.getName(), false);
      }

      camera.flushUpdates().stream()
          .map(this::gaugeTrust)
          .filter(Optional::isPresent)
          .map(Optional::get)
          .forEach(
              sample -> {
                timerSinceLastSample.restart();
                visionSender.send(sample);
              });

      seenTags.addAll(camera.getSeenTags());

      Tracer.endTrace();
    }

    Pose3d[] tagLoc3d =
        seenTags.stream()
            .map(i -> FieldConstants.APRIL_TAG_FIELD.getTagPose(i))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toArray(Pose3d[]::new);
    Pose2d[] tagLoc2d = new Pose2d[tagLoc3d.length];
    for (int i = 0; i < tagLoc3d.length; i++) {
      tagLoc2d[i] = tagLoc3d[i].toPose2d();
    }
    log("seenTags", tagLoc3d);
    GlobalField.setObject("SeenTags", tagLoc2d);
    seenTags.clear();

    Tracer.endTrace();
  }
}
