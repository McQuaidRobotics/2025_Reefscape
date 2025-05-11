package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import igknighters.subsystems.vision.Vision.VisionSample;
import igknighters.util.plumbing.Channel;
import igknighters.util.plumbing.Channel.Receiver;
import igknighters.util.plumbing.Channel.Sender;
import igknighters.util.plumbing.Channel.ThreadSafetyMarker;
import java.util.List;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.GlobalField;
import monologue.Logged;
import wayfinder.controllers.Framework.Assessor;
import wayfinder.poseEst.TwistyPoseEst;
import wayfinder.poseEst.TwistyPoseEst.VisionScalars;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Tracer;

public class Localizer implements Logged, Assessor<Pose2d, FieldSpeeds> {
  private static final VisionScalars VISION_SCALARS = new VisionScalars(0.9, 0.05, 1.0);

  public static boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
    if (Math.abs(toleranceRadians) > Math.PI) {
      return true;
    }
    double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
    // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi], as pre-checked
    // above.
    return dot > Math.cos(toleranceRadians);
  }

  private final Channel<VisionSample> visionDataChannel = new Channel<>(new VisionSample[0]);
  private final Channel<SwerveDriveSample> swerveDataChannel =
      new Channel<>(new SwerveDriveSample[0]);

  private final Receiver<VisionSample> visionDataReceiver =
      visionDataChannel.openReceiver(8, ThreadSafetyMarker.SEQUENTIAL);
  private final Receiver<SwerveDriveSample> swerveDataReceiver =
      swerveDataChannel.openReceiver(32, ThreadSafetyMarker.CONCURRENT);

  private final TwistyPoseEst poseEstimator;

  private final SwerveModuleState[] moduleStates = {
    new SwerveModuleState(0.0, Rotation2d.kZero),
    new SwerveModuleState(0.0, Rotation2d.kZero),
    new SwerveModuleState(0.0, Rotation2d.kZero),
    new SwerveModuleState(0.0, Rotation2d.kZero)
  };

  @Log(key = "pose")
  private Pose2d latestPose = FieldConstants.POSE2D_CENTER;

  @Log(key = "speed")
  private FieldSpeeds latestSpeed = FieldSpeeds.kZero;

  private double resetTime = 0.0;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(kSwerve.MODULE_CHASSIS_LOCATIONS);

  public Localizer() {
    poseEstimator = new TwistyPoseEst();

    GlobalField.setObject(
        "AprilTags",
        FieldConstants.APRIL_TAG_FIELD.getTags().stream()
            .map(tag -> tag.pose.toPose2d())
            .toArray(Pose2d[]::new));
  }

  public Sender<VisionSample> visionDataSender() {
    return visionDataChannel.sender();
  }

  public Sender<SwerveDriveSample> swerveDataSender() {
    return swerveDataChannel.sender();
  }

  public Receiver<SwerveDriveSample> swerveDataReceiver() {
    return swerveDataReceiver.fork(32, ThreadSafetyMarker.CONCURRENT);
  }

  public void reset(Pose2d pose) {
    resetTime = Timer.getFPGATimestamp() + 0.02;
    poseEstimator.resetPose(pose);
    latestPose = pose;
    GlobalField.setObject("Robot", pose);
  }

  public void update() {
    Tracer.startTrace("SwerveSamples");
    final SwerveDriveSample[] swerveSamples = log("swerveSamples", swerveDataReceiver.recvAll());
    for (final SwerveDriveSample sample : swerveSamples) {
      if (sample.timestamp() < resetTime) {
        continue;
      }
      poseEstimator.addDriveSample(
          kinematics, sample.modulePositions(), sample.gyroYaw(), sample.timestamp(), 1.0);
    }
    if (swerveSamples.length > 0) {
      var lastSample = swerveSamples[swerveSamples.length - 1];
      for (int i = 0; i < moduleStates.length; i++) {
        var sampleState = lastSample.modulePositions()[i];
        moduleStates[i].angle = sampleState.angle;
        moduleStates[i].speedMetersPerSecond = sampleState.speedMetersPerSecond;
      }
      latestSpeed =
          Speeds.fromRobotRelative(kinematics.toChassisSpeeds(moduleStates))
              .asFieldRelative(lastSample.gyroYaw());
    }
    Tracer.endTrace();
    Tracer.startTrace("VisionSamples");
    final List<VisionSample> unsortedVisionSamples =
        List.of(log("visionSamples", visionDataReceiver.recvAll()));
    final List<VisionSample> visionSamples =
        unsortedVisionSamples.stream()
            .sorted((a, b) -> Double.compare(a.timestamp(), b.timestamp()))
            .toList();
    double sumLatency = 0.0;
    for (final VisionSample sample : visionSamples) {
      if (sample.timestamp() < resetTime) {
        continue;
      }
      poseEstimator.addVisionSample(
          VISION_SCALARS, sample.pose(), sample.timestamp(), sample.trust());
    }
    log("visionLatency", sumLatency / visionSamples.size());
    Tracer.endTrace();

    Tracer.startTrace("prune");
    poseEstimator.prune(0.25);
    Tracer.endTrace();

    latestPose = Tracer.traceFunc("readEstPose", poseEstimator::getEstimatedPose);
    GlobalField.setObject("Robot", latestPose);
  }

  public Optional<Pose2d> pose(double secondsAgo) {
    return poseEstimator.getEstimatedPoseFromPast(secondsAgo);
  }

  public Pose2d pose() {
    return latestPose;
  }

  public Rotation2d rotation() {
    return latestPose.getRotation();
  }

  public Translation2d translation() {
    return latestPose.getTranslation();
  }

  public FieldSpeeds speed() {
    return latestSpeed;
  }

  public Trigger near(Rotation2d target, double toleranceRadians) {
    return new Trigger(() -> withinTolerance(rotation(), target, toleranceRadians));
  }

  public Trigger near(Translation2d target, double toleranceMeters) {
    return new Trigger(() -> translation().getDistance(target) < toleranceMeters);
  }

  public Trigger near(Pose2d target, double toleranceMeters, double toleranceRadians) {
    return near(target.getTranslation(), toleranceMeters)
        .and(near(target.getRotation(), toleranceRadians));
  }

  public Trigger slowerThan(double speed) {
    return new Trigger(() -> speed().magnitude() < speed);
  }

  public Trigger fasterThan(double speed) {
    return new Trigger(() -> speed().magnitude() > speed);
  }

  @Override
  public Pose2d getMeasurement() {
    return latestPose;
  }

  @Override
  public FieldSpeeds getRate() {
    return latestSpeed;
  }
}