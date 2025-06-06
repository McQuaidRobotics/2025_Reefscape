package igknighters.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.constants.ConstValues;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kDriveMotor;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kSteerMotor;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kWheel;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import wayfinder.setpointGenerator.AdvancedSwerveModuleState;

public class SwerveModuleSim extends SwerveModule {
  private static final DCMotor MOTOR = DCMotor.getFalcon500(1);
  private final FlywheelSim driveSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(MOTOR, 0.025, kDriveMotor.GEAR_RATIO), MOTOR);
  private final FlywheelSim angleSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(MOTOR, 0.004, kSteerMotor.GEAR_RATIO), MOTOR);

  private final PIDController driveFeedback =
      new PIDController(kDriveMotor.kP, kDriveMotor.kI, kDriveMotor.kD, ConstValues.PERIODIC_TIME);
  private final PIDController angleFeedback =
      new PIDController(kSteerMotor.kP, kSteerMotor.kI, kSteerMotor.kD, ConstValues.PERIODIC_TIME);

  public final int moduleId;

  public SwerveModuleSim(final int moduleId, SimSwerveOdometryThread odoThread) {
    super("SwerveModule[" + moduleId + "]");
    this.moduleId = moduleId;

    angleFeedback.enableContinuousInput(-Math.PI, Math.PI);

    super.steerAbsoluteRads = Units.rotationsToRadians(Math.random());

    odoThread.addModulePositionSupplier(moduleId, this::getCurrentPosition);

    BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "] initialized (sim)");
  }

  private double driveRotationsToMeters(double rotations) {
    return rotations * kWheel.CIRCUMFERENCE;
  }

  private double driveRadiansToMeters(double radians) {
    return driveRotationsToMeters(radians / (2.0 * Math.PI));
  }

  public void setDesiredState(AdvancedSwerveModuleState desiredState) {
    super.controlledLastCycle = true;
    desiredState.optimize(getAngle());
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(super.drivePositionMeters, getAngle());
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(super.driveVeloMPS, getAngle());
  }

  @Override
  public int getModuleId() {
    return this.moduleId;
  }

  private Rotation2d getAngle() {
    return new Rotation2d(super.steerAbsoluteRads);
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
            ? new Rotation2d(super.steerAbsoluteRads)
            : desiredState.angle;
    super.targetSteerAbsoluteRads = angle.getRadians();

    var angleAppliedVolts =
        MathUtil.clamp(
            angleFeedback.calculate(getAngle().getRadians(), angle.getRadians()),
            -RobotController.getBatteryVoltage(),
            RobotController.getBatteryVoltage());
    angleSim.setInputVoltage(angleAppliedVolts);

    super.steerVolts = angleAppliedVolts;
    super.steerAbsoluteRads = angle.getRadians();
  }

  private void setSpeed(SwerveModuleState desiredState) {
    super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;

    desiredState.speedMetersPerSecond *= Math.cos(angleFeedback.getError());

    double velocityRadPerSec = desiredState.speedMetersPerSecond / kWheel.RADIUS;
    var driveAppliedVolts =
        MathUtil.clamp(
            driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec),
            -1.0 * RobotController.getBatteryVoltage(),
            RobotController.getBatteryVoltage());
    driveSim.setInputVoltage(driveAppliedVolts);

    super.driveVolts = driveAppliedVolts;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !super.controlledLastCycle) {
      setVoltageOut(0.0, getAngle());
    }
    super.controlledLastCycle = false;

    driveSim.update(ConstValues.PERIODIC_TIME);
    angleSim.update(ConstValues.PERIODIC_TIME);

    super.drivePositionMeters +=
        driveRadiansToMeters(driveSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME);

    double angleDiffRad = angleSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME;
    super.steerAbsoluteRads += angleDiffRad;

    while (super.steerAbsoluteRads < 0) {
      super.steerAbsoluteRads += 2 * Math.PI;
    }
    while (super.steerAbsoluteRads > 2 * Math.PI) {
      super.steerAbsoluteRads -= 2 * Math.PI;
    }

    super.steerVeloRadPS = angleSim.getAngularVelocityRadPerSec();
    super.steerAmps = angleSim.getCurrentDrawAmps();

    super.driveVeloMPS = driveRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0);
    super.driveAmps = driveSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltageOut(double volts, Rotation2d angle) {
    super.controlledLastCycle = true;
    super.driveVolts = volts;
    super.steerAbsoluteRads = angle.getRadians();
    super.targetSteerAbsoluteRads = angle.getRadians();
  }
}
