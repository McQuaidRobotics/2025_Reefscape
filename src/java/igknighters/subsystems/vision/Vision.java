package igknighters.subsystems.vision;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import igknighters.subsystems.vision.VisionConstants.kVision;
import igknighters.subsystems.vision.camera.Camera;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.subsystems.vision.camera.CameraDisabled;
import igknighters.subsystems.vision.camera.CameraRealPhoton;
import igknighters.subsystems.vision.camera.CameraSimPhoton;
import igknighters.util.plumbing.Channel.Sender;
import java.util.HashSet;
import java.util.Optional;
import monologue.Annotations.IgnoreLogged;
import monologue.GlobalField;
import monologue.ProceduralStructGenerator;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Tracer;

public class Vision implements SharedSubsystem {
  static {
    OpenCvLoader.forceStaticLoad();
  }

  private final SharedState shared;
  @IgnoreLogged private final Localizer localizer;

  private final Sender<VisionSample> visionSender;

  private final Camera[] cameras;

  private final Timer timerSinceLastSample = new Timer();
  private final HashSet<Integer> seenTags = new HashSet<>();

  public record VisionUpdate(Pose2d pose, double timestamp, double trustScalar)
      implements StructSerializable {

    private static final VisionUpdate kEmpty = new VisionUpdate(Pose2d.kZero, 0.0, 1.0);

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

  private Camera makeCamera(CameraConfig config, SimCtx simCtx) {
    try {
      if (Robot.isSimulation()) {
        return new CameraSimPhoton(config, simCtx);
      } else {
        return new CameraRealPhoton(config);
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
  }

  private Optional<VisionSample> gaugeTrust(final VisionUpdate update) {
    double trust = update.trustScalar();

    // Completely arbitrary values for the velocity thresholds.
    // When the robot is moving fast there can be paralaxing and motion blur
    // that can cause the vision system to be less accurate, reduce the trust due to this
    FieldSpeeds velo = shared.fieldSpeeds;
    trust *= kVision.LINEAR_VELOCITY_TRUST_COEFFICIENT.lerp(velo.magnitude());
    trust *= kVision.ANGULAR_VELOCITY_TRUST_COEFFICIENT.lerp(velo.omega());

    return Optional.of(new VisionSample(update.pose(), update.timestamp(), trust));
  }

  public double timeSinceLastSample() {
    return timerSinceLastSample.get();
  }

  @Override
  public void periodic() {
    Tracer.startTrace("VisionPeriodic");
    for (final Camera camera : cameras) {
      Tracer.startTrace(camera.getName() + "Periodic");

      try {
        camera.periodic();
      } catch (Exception e) {
        DriverStation.reportError("Error in camera " + camera.getName(), e.getStackTrace());
      }

      camera.flushUpdates().stream()
          .map(this::gaugeTrust)
          .filter(Optional::isPresent)
          .map(Optional::get)
          .forEach(
              sample -> {
                timerSinceLastSample.restart();
                visionSender.send(sample);
                log("trust", sample.trust());
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
