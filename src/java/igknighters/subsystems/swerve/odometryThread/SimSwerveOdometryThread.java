package igknighters.subsystems.swerve.odometryThread;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import igknighters.subsystems.swerve.odometryThread.SwerveDriveSample.SwerveModuleSuperState;
import igknighters.util.plumbing.Channel.Sender;
import java.util.function.Supplier;

public class SimSwerveOdometryThread extends SwerveOdometryThread {
  private final Notifier notifier;

  @SuppressWarnings("unchecked")
  private final Supplier<SwerveModulePosition>[] positionSuppliers = new Supplier[MODULE_COUNT];

  @SuppressWarnings("unchecked")
  private final Supplier<SwerveModuleState>[] stateSuppliers = new Supplier[MODULE_COUNT];

  private Supplier<Rotation2d> rotationSupplier = Rotation2d::new;
  private Supplier<double[]> accelerationSupplier = () -> new double[] {0.0, 0.0};

  public SimSwerveOdometryThread(int hz, Sender<SwerveDriveSample> swerveDataSender) {
    super(hz, swerveDataSender);
    notifier = new Notifier(this::run);
    notifier.setName("SwerveOdometry");
  }

  public void addModuleSupplier(
      int moduleId, Supplier<SwerveModulePosition> posSup, Supplier<SwerveModuleState> stateSup) {
    positionSuppliers[moduleId] = posSup;
    stateSuppliers[moduleId] = stateSup;
  }

  public void addRotationSupplier(Supplier<Rotation2d> sup) {
    rotationSupplier = sup;
  }

  public void addAccelerationSupplier(Supplier<double[]> sup) {
    accelerationSupplier = sup;
  }

  private SwerveModuleSuperState[] getModulePositions() {
    SwerveModuleSuperState[] positions = new SwerveModuleSuperState[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      SwerveModulePosition position = positionSuppliers[i].get();
      SwerveModuleState state = stateSuppliers[i].get();
      positions[i] = new SwerveModuleSuperState(position, state.speedMetersPerSecond);
    }
    return positions;
  }

  private void run() {
    if (DriverStation.isTestEnabled()) {
      return;
    }
    long startTime = RobotController.getFPGATime();
    var accel = accelerationSupplier.get();
    swerveDataSender.send(
        new SwerveDriveSample(
            getModulePositions(),
            rotationSupplier.get(),
            Math.hypot(accel[0], accel[1]),
            Timer.getFPGATimestamp()));
    updateTimeMicros.set(RobotController.getFPGATime() - startTime);
  }

  @Override
  public void start() {
    notifier.startPeriodic(1.0 / ((double) hz));
    isRunning.set(true);
  }
}
