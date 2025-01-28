package igknighters;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kRobotIntrinsics;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.constants.FieldConstants;
import igknighters.util.plumbing.Channel.Receiver;
import monologue.GlobalField;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import sham.ShamArena;
import sham.ShamRobot;
import sham.ShamSwerve;
import sham.configs.ShamGyroConfig;
import sham.configs.ShamMechanismConfig;
import sham.configs.ShamSwerveConfig;
import sham.configs.ShamSwerveModuleConfig;
import sham.configs.ShamSwerveModuleConfig.WheelCof;
import sham.seasonspecific.Reefscape;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

/**
 * An object containing sim-specific objects and configurations.
 *
 * <p>This object should be treated similar to an optional, where the simulation context is only
 * usable if {@link #isActive()} returns true.
 */
public class SimCtx {
  private final boolean isSimulation;

  // all fields below this point will be null if isSimulation is false
  private final ShamArena arena;
  private final ShamRobot<ShamSwerve> simRobot;

  private final VisionSystemSim aprilTagSim;
  private final VisionSystemSim objectDetectionSim;
  private final TargetModel gpTargetModel =
      new TargetModel(
          Units.inchesToMeters(14.0), Units.inchesToMeters(14.0), Units.inchesToMeters(2.0));

  private final Receiver<Pose2d> resetReceiver;

  private final ShamMechanismConfig driveMotorCfg =
      new ShamMechanismConfig(new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1))
          .withFriction(Volts.of(kDriveMotor.kS), Volts.of(kDriveMotor.kS * 0.8))
          .withGearRatio(GearRatio.reduction(kSwerve.DRIVE_GEAR_RATIO))
          .withNoise(0.00)
          .withRotorInertia(KilogramSquareMeters.of(0.003));
  private final ShamMechanismConfig steerMotorCfg =
      new ShamMechanismConfig(new DCMotorExt(DCMotor.getFalcon500Foc(1), 1))
          .withFriction(Volts.of(kSteerMotor.kS), Volts.of(kSteerMotor.kS * 0.8))
          .withGearRatio(GearRatio.reduction(kSwerve.STEER_GEAR_RATIO))
          .withNoise(0.00)
          .withRotorInertia(KilogramSquareMeters.of(0.02));
  private final ShamSwerveModuleConfig moduleCfg =
      new ShamSwerveModuleConfig(
          driveMotorCfg, steerMotorCfg, WheelCof.BLACK_NITRILE.cof, kSwerve.WHEEL_DIAMETER / 2.0);
  private final ShamSwerveConfig swerveConfig =
      new ShamSwerveConfig(
          kRobotIntrinsics.MASS,
          kRobotIntrinsics.MOMENT_OF_INERTIA,
          kRobotIntrinsics.CHASSIS_WIDTH,
          kRobotIntrinsics.CHASSIS_WIDTH,
          kSwerve.MODULE_CHASSIS_OFFSETS,
          moduleCfg,
          ShamGyroConfig.ofPigeon2());

  public SimCtx(Localizer localizer, boolean isSim) {
    isSimulation = isSim;
    resetReceiver = localizer.poseResetsReceiver();
    if (isSimulation) {
      // arena = new ShamArena(new FieldMap(), ConstValues.PERIODIC_TIME, 5) {};
      arena = new Reefscape.ReefscapeShamArena(Seconds.of(ConstValues.PERIODIC_TIME), 5);
      simRobot = new ShamRobot<>(arena, "User", swerveConfig, 1);
      aprilTagSim = new VisionSystemSim("AprilTags");
      aprilTagSim.addAprilTags(FieldConstants.APRIL_TAG_FIELD);
      objectDetectionSim = new VisionSystemSim("ObjectDetection");
    } else {
      arena = null;
      simRobot = null;
      aprilTagSim = null;
      objectDetectionSim = null;
    }
  }

  public boolean isActive() {
    return isSimulation;
  }

  public ShamArena arena() {
    return arena;
  }

  public ShamRobot<ShamSwerve> robot() {
    return simRobot;
  }

  public VisionSystemSim aprilTagSim() {
    return aprilTagSim;
  }

  public VisionSystemSim objectDetectionSim() {
    return objectDetectionSim;
  }

  public void update() {
    if (isSimulation) {
      if (resetReceiver.hasData()) {
        final var poses = resetReceiver.recvAll();
        robot().getDriveTrain().setChassisWorldPose(poses[poses.length - 1], true);
      }
      arena.simulationPeriodic();
      final Pose2d robotPose = simRobot.getDriveTrain().getChassisWorldPose();
      GlobalField.setObject("SimRobot", robotPose);
      aprilTagSim.update(robotPose);
      objectDetectionSim.update(robotPose);
      objectDetectionSim.clearVisionTargets();
      final var objectTargets =
          arena
              .gamePieces()
              .map(gp -> new VisionTargetSim(gp.pose(), gpTargetModel))
              .toArray(VisionTargetSim[]::new);
      objectDetectionSim.addVisionTargets("gamepieces", objectTargets);
    }
  }
}
