package igknighters.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import igknighters.DeviceManager;
import igknighters.subsystems.swerve.SwerveConstants.kGyro;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import monologue.Annotations.IgnoreLogged;

public class GyroReal extends Gyro {

  private final Pigeon2 gyro;
  private final BaseStatusSignal rollSignal, pitchSignal;

  @IgnoreLogged private final RealSwerveOdometryThread odoThread;

  public GyroReal(DeviceManager deviceManager, RealSwerveOdometryThread odoThread) {
    this.odoThread = odoThread;

    gyro = new Pigeon2(kGyro.PIGEON_ID, kSwerve.CANBUS);
    Pigeon2Configuration cfg = new Pigeon2Configuration();
    cfg.MountPose.MountPoseRoll = 180.0;
    deviceManager.bringUp(this, "imu", gyro, cfg);

    rollSignal = gyro.getRoll();
    pitchSignal = gyro.getPitch();

    odoThread.addGyroStatusSignals(
        gyro.getYaw().clone(),
        gyro.getAngularVelocityZWorld().clone(),
        gyro.getAccelerationX().clone(),
        gyro.getAccelerationY().clone());

    gyro.optimizeBusUtilization(0.0, 1.0);

    BootupLogger.bootupLog("    Gyro initialized (real)");
  }

  @Override
  public double getPitchRads() {
    return super.pitchRads;
  }

  @Override
  public double getRollRads() {
    return super.rollRads;
  }

  @Override
  public double getYawRads() {
    return super.yawRads;
  }

  @Override
  public void setYawRads(double yawRads) {
    gyro.setYaw(Units.radiansToDegrees(yawRads));
  }

  @Override
  public void periodic() {
    super.pitchRads = Units.degreesToRadians(pitchSignal.getValueAsDouble());
    super.rollRads = Units.degreesToRadians(rollSignal.getValueAsDouble());
    super.yawRads = odoThread.getGyroYaw();
    super.yawVelRadsPerSec = odoThread.getGyroYawRate();
  }
}
