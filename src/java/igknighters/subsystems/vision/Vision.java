package igknighters.subsystems.vision;

import edu.wpi.first.math.MathUtil;
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
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kVision;
import igknighters.constants.FieldConstants;
import igknighters.constants.RobotConfig;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.vision.camera.Camera;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.subsystems.vision.camera.CameraDisabled;
import igknighters.subsystems.vision.camera.CameraRealPhoton;
import igknighters.subsystems.vision.camera.CameraSimPhoton;
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
  @IgnoreLogged private final Localizer localizer;

  private final Sender<VisionSample> visionSender;

  private final Camera[] cameras;

  private final HashSet<Integer> seenTags = new HashSet<>();

  public record VisionUpdateFlaws(
      boolean extremeJitter,
      boolean outOfBounds,
      boolean outOfRange,
      boolean infeasibleZValue,
      boolean infeasiblePitchValue,
      boolean infeasibleRollValue,
      boolean sketchyTags,
      boolean singleTag)
      implements StructSerializable {

    private static final VisionUpdateFlaws kEmpty =
        new VisionUpdateFlaws(false, false, false, false, false, false, false, false);

    public static VisionUpdateFlaws empty() {
      return kEmpty;
    }

    public static VisionUpdateFlaws solve(
        Pose3d pose,
        Pose3d lastPose,
        double time,
        double avgDistance,
        List<Integer> tagsList,
        List<Integer> sketchyTagsList) {
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
      boolean outOfRange = avgDistance > 5.5;
      boolean singleTag = tagsList.size() == 1;
      boolean sketchyTags = tagsList.stream().anyMatch(sketchyTagsList::contains);
      return new VisionUpdateFlaws(
          extremeJitter,
          outOfBounds,
          outOfRange,
          infeasibleZValue,
          infeasiblePitchValue,
          infeasibleRollValue,
          sketchyTags,
          singleTag);
    }

    public static final Struct<VisionUpdateFlaws> struct =
        ProceduralStructGenerator.genRecord(VisionUpdateFlaws.class);
  }

  public record VisionUpdate(Pose2d pose, double timestamp, VisionUpdateFlaws faults)
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
    return localizer.pose(Timer.getFPGATimestamp() - timestamp).getRotation();
  }

  private Camera makeCamera(CameraConfig config, SimCtx simCtx) {
    try {
      if (Robot.isSimulation()) {
        return new CameraSimPhoton(config, simCtx, this::rotationAtTimestamp);
      } else {
        return new CameraRealPhoton(config, this::rotationAtTimestamp);
      }
    } catch (Exception e) {
      // if the camera fails to initialize, return a disabled camera to not crash the code
      DriverStation.reportError("Failed to initialize camera: " + config.cameraName(), false);
      return new CameraDisabled(config.cameraName(), config.cameraTransform());
    }
  }

  public Vision(final Localizer localizer, final SimCtx simCtx) {
    this.localizer = localizer;
    final var configs = kVision.CameraConfigs.forRobot(RobotConfig.getRobotID());
    this.cameras = new Camera[configs.length];
    for (int i = 0; i < configs.length; i++) {
      this.cameras[i] = makeCamera(configs[i], simCtx);
    }

    visionSender = localizer.visionDataSender();
  }

  private Optional<VisionSample> gaugeTrust(final VisionUpdate update) {
    final VisionUpdateFlaws faults = update.faults();

    // These are "fatal" faults that should always invalidate the update
    if ((faults.sketchyTags && faults.singleTag) || faults.outOfBounds || faults.extremeJitter) {
      return Optional.empty();
    }

    double trust = kVision.ROOT_TRUST * TunableValues.getDouble("visionTrustScaler", 1.0).value();

    // If the average distance of the tags is too far away reduce the trust
    if (faults.outOfRange()) {
      trust /= 2.0;
    }

    // If there is a sketchy tag reduce the trust
    if (faults.sketchyTags) {
      trust /= 2.0;
    }

    // Completely arbitrary values for the velocity thresholds.
    // When the robot is moving fast there can be paralaxing and motion blur
    // that can cause the vision system to be less accurate, reduce the trust due to this
    FieldSpeeds velo = localizer.speeds();
    if (Math.hypot(velo.vx(), velo.vy()) > kSwerve.MAX_DRIVE_VELOCITY / 2.0) {
      trust /= 2.0;
    }
    if (velo.omega() > kSwerve.MAX_ANGULAR_VELOCITY / 3.0) {
      trust /= 2.0;
    }

    // If the vision rotation varies significantly from the gyro rotation reduce the trust
    Rotation2d rotation = localizer.pose().getRotation();
    if (Math.abs(
            MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(update.pose().getRotation().getRadians()))
        > Math.toRadians(5.0)) {
      trust /= 2.0;
    }

    if (faults.infeasibleZValue || faults.infeasiblePitchValue || faults.infeasibleRollValue) {
      trust /= 2.0;
    }

    return Optional.of(new VisionSample(update.pose(), update.timestamp(), trust));
  }

  @Override
  public void periodic() {
    Tracer.startTrace("VisionPeriodic");

    for (final Camera camera : cameras) {
      Tracer.startTrace(camera.getName() + "Periodic");
      try {
        camera.periodic();
      } catch (Exception e) {
        DriverStation.reportError("Error in camera " + camera.getName(), false);
      }

      camera.flushUpdates().stream()
          .map(this::gaugeTrust)
          .filter(Optional::isPresent)
          .map(Optional::get)
          .forEach(visionSender::send);

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
