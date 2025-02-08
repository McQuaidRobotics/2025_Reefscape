package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import igknighters.subsystems.vision.Vision.VisionSample;
import igknighters.util.plumbing.Channel;
import igknighters.util.plumbing.Channel.Receiver;
import igknighters.util.plumbing.Channel.Sender;
import igknighters.util.plumbing.Channel.ThreadSafetyMarker;
import java.util.List;
import monologue.Annotations.Log;
import monologue.GlobalField;
import monologue.Logged;
import wayfinder.poseEst.TwistyPoseEst;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Tracer;

public class Localizer implements Logged {

  private final Channel<VisionSample> visionDataChannel = new Channel<>(new VisionSample[0]);
  private final Channel<SwerveDriveSample> swerveDataChannel =
      new Channel<>(new SwerveDriveSample[0]);

  private final Receiver<VisionSample> visionDataReceiver =
      visionDataChannel.openReceiver(8, ThreadSafetyMarker.CONCURRENT);
  private final Receiver<SwerveDriveSample> swerveDataReceiver =
      swerveDataChannel.openReceiver(32, ThreadSafetyMarker.CONCURRENT);

  private final TwistyPoseEst poseEstimator;

  @Log(key = "pose")
  private Pose2d latestPose = FieldConstants.POSE2D_CENTER;

  @Log(key = "speeds")
  private FieldSpeeds latestSpeeds = FieldSpeeds.kZero;

  private final Channel<Pose2d> poseResetsChannel = new Channel<>(new Pose2d[0]);
  private final Sender<Pose2d> poseResetsSender = poseResetsChannel.sender();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(kSwerve.MODULE_CHASSIS_OFFSETS);

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

  public Receiver<Pose2d> poseResetsReceiver() {
    return poseResetsChannel.openReceiver(8, ThreadSafetyMarker.CONCURRENT);
  }

  public void reset(Pose2d pose) {
    poseEstimator.resetPose(pose);
    poseResetsSender.send(pose);
  }

  public void update() {
    Tracer.startTrace("SwerveSamples");
    final SwerveDriveSample[] swerveSamples = log("swerveSamples", swerveDataReceiver.recvAll());
    for (final SwerveDriveSample sample : swerveSamples) {
      poseEstimator.addDriveSample(
          kinematics, sample.modulePositions(), sample.gyroYaw(), sample.timestamp(), 1.0);
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
      poseEstimator.addVisionSample(sample.pose(), sample.timestamp(), sample.trust());
    }
    log("visionLatency", sumLatency / visionSamples.size());
    Tracer.endTrace();

    Tracer.startTrace("Prune");
    poseEstimator.prune(0.25);
    Tracer.endTrace();

    latestPose = Tracer.traceFunc("ReadEstPose", poseEstimator::getEstimatedPose);
    GlobalField.setObject("Robot", latestPose);

    Pose2d poseFromABitAgo = poseEstimator.getEstimatedPoseFromPast(0.05);
    Twist2d twist = poseFromABitAgo.log(latestPose);
    latestSpeeds = Speeds.fromFieldRelative(twist.dx / 0.05, twist.dy / 0.05, twist.dtheta / 0.05);
  }

  public Pose2d pose() {
    return latestPose;
  }

  public Pose2d pose(double secondsAgo) {
    return poseEstimator.getEstimatedPoseFromPast(secondsAgo);
  }

  public Rotation2d rotation() {
    return latestPose.getRotation();
  }

  public FieldSpeeds speeds() {
    return latestSpeeds;
  }

  public Translation2d translation() {
    return latestPose.getTranslation();
  }
}
