package igknighters.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kWheel;
import igknighters.subsystems.swerve.odometryThread.SwerveDriveSample.SwerveModuleSuperState;
import igknighters.util.plumbing.Channel.Sender;
import java.util.concurrent.atomic.AtomicLong;

public class RealSwerveOdometryThread extends SwerveOdometryThread {
  private static final int SIGNALS_PER_MODULE = 3;

  private final Thread thread;
  private final BaseStatusSignal[] signals =
      new BaseStatusSignal[(MODULE_COUNT * SIGNALS_PER_MODULE) + 4];

  protected final MedianFilter peakRemover = new MedianFilter(3);
  protected final LinearFilter lowPass = LinearFilter.movingAverage(50);

  /**
   * An array that holds [module1Pos, module1Velo, module2Pos, ...] Doesn't include steer motor
   * values, the module just uses the cancoder to get past concurrency issues.
   */
  protected final AtomicLong[] moduleStates = new AtomicLong[MODULE_COUNT * SIGNALS_PER_MODULE];

  protected final AtomicLong gyroYaw = new AtomicLong();

  protected boolean enableLatencyCompensation = false;

  private double getAtomicDouble(AtomicLong[] array, int index) {
    return Double.longBitsToDouble(array[index].get());
  }

  public RealSwerveOdometryThread(int hz, Sender<SwerveDriveSample> swerveDataSender) {
    super(hz, swerveDataSender);
    this.thread = new Thread(this::run, "OdometryThread");
    for (int i = 0; i < MODULE_COUNT * 2; i++) {
      moduleStates[i] = new AtomicLong();
    }
  }

  public void addModuleStatusSignals(
      int moduleId,
      BaseStatusSignal drivePosition,
      BaseStatusSignal driveVelocity,
      BaseStatusSignal anglePosition) {
    BaseStatusSignal.setUpdateFrequencyForAll(hz, drivePosition, driveVelocity, anglePosition);
    int offset = 4 * moduleId;
    signals[offset + 0] = drivePosition;
    signals[offset + 1] = driveVelocity;
    signals[offset + 2] = anglePosition;
  }

  public void addGyroStatusSignals(
      BaseStatusSignal yaw, BaseStatusSignal xAccel, BaseStatusSignal yAccel) {
    BaseStatusSignal.setUpdateFrequencyForAll(hz, yaw, xAccel, yAccel);
    int moduleSignalCount = MODULE_COUNT * SIGNALS_PER_MODULE;
    signals[moduleSignalCount + 0] = yaw;
    signals[moduleSignalCount + 1] = xAccel;
    signals[moduleSignalCount + 2] = yAccel;
  }

  private SwerveModuleSuperState[] getModulePositions() {
    SwerveModuleSuperState[] positions = new SwerveModuleSuperState[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      int offset = i * SIGNALS_PER_MODULE;
      positions[i] =
          new SwerveModuleSuperState(
              signals[offset + 0].getValueAsDouble() * kWheel.CIRCUMFERENCE,
              signals[offset + 1].getValueAsDouble() * kWheel.CIRCUMFERENCE,
              Rotation2d.fromRotations(signals[offset + 2].getValueAsDouble()));
    }
    return positions;
  }

  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(signals[MODULE_COUNT * SIGNALS_PER_MODULE].getValueAsDouble());
  }

  private double getDataLatency() {
    double totalLatency = 0.0;
    for (var signal : signals) {
      totalLatency += signal.getTimestamp().getLatency();
    }
    return totalLatency / (double) signals.length;
  }

  private double getAcceleration() {
    return Math.hypot(
        signals[(MODULE_COUNT * 4) + 2].getValueAsDouble(),
        signals[(MODULE_COUNT * 4) + 3].getValueAsDouble());
  }

  private void run() {
    isRunning.set(true);
    try {
      Threads.setCurrentThreadPriority(true, 1);

      while (this.isRunning.get()) {
        long startTime = RobotController.getFPGATime();
        BaseStatusSignal.waitForAll(2.0 / hz, signals);
        long elapsedTime = RobotController.getFPGATime() - startTime;

        updateTimeMicros.set((long) lowPass.calculate(peakRemover.calculate(elapsedTime)));

        if (DriverStation.isTestEnabled()) {
          continue;
        }

        for (int i = 0; i < MODULE_COUNT; i++) {
          int positionOffset = 4 * i;
          int veloOffset = positionOffset + 1;

          moduleStates[i * 2].set(
              Double.doubleToLongBits(signals[positionOffset].getValueAsDouble()));
          moduleStates[(i * 2) + 1].set(
              Double.doubleToLongBits(signals[veloOffset].getValueAsDouble()));
        }

        int moduleSignalCount = MODULE_COUNT * SIGNALS_PER_MODULE;
        gyroYaw.set(
            Double.doubleToLongBits(
                Units.degreesToRadians(signals[moduleSignalCount].getValueAsDouble())));

        swerveDataSender.send(
            new SwerveDriveSample(
                getModulePositions(),
                getGyroRotation(),
                getAcceleration(),
                Timer.getFPGATimestamp() - log("latency", getDataLatency())));
      }
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    } finally {
      isRunning.set(false);
    }
  }

  @Override
  public void start() {
    thread.start();
  }

  public double getModulePosition(int moduleId) {
    return getAtomicDouble(moduleStates, moduleId * 2);
  }

  public double getModuleVelocity(int moduleId) {
    return getAtomicDouble(moduleStates, (moduleId * 2) + 1);
  }

  public double getGyroYaw() {
    return Double.longBitsToDouble(gyroYaw.get());
  }
}
