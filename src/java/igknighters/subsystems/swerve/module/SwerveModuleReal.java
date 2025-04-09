package igknighters.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.DeviceManager;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kDriveMotor;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kSteerMotor;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kWheel;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import monologue.Annotations.IgnoreLogged;
import wayfinder.setpointGenerator.AdvancedSwerveModuleState;

public class SwerveModuleReal extends SwerveModule {
  private final TalonFX driveMotor;
  private final BaseStatusSignal driveVoltSignal, driveAmpSignal;
  private final VelocityVoltage driveMotorClosedReq;

  private final TalonFX steerMotor;
  private final BaseStatusSignal steerVoltSignal, steerAmpSignal;
  private final MotionMagicVoltage steerMotorReq = new MotionMagicVoltage(0).withUpdateFreqHz(0);

  private final CANcoder steerEncoder;
  private final BaseStatusSignal steerAbsoluteSignal, steerAbsoluteVeloSignal;

  public final int moduleId;

  @IgnoreLogged private final RealSwerveOdometryThread odoThread;

  private Rotation2d lastAngle = new Rotation2d();

  public SwerveModuleReal(final int moduleId, DeviceManager deviceManager, RealSwerveOdometryThread odoThread) {
    super("SwerveModule[" + moduleId + "]");
    this.odoThread = odoThread;

    this.moduleId = moduleId;
    double rotationOffset = super.getOffset(moduleId);

    driveMotor = new TalonFX((moduleId * 2) + 1, kSwerve.CANBUS);
    steerMotor = new TalonFX((moduleId * 2) + 2, kSwerve.CANBUS);
    steerEncoder = new CANcoder((moduleId * 2) + 2, kSwerve.CANBUS);

    // CANRetrier.retryStatusCode(
    //     () -> driveMotor.getConfigurator().apply(driveMotorConfig(), 1.0), 5);
    // CANRetrier.retryStatusCode(
    //     () -> steerMotor.getConfigurator().apply(steerMotorConfig(steerEncoder.getDeviceID()), 1.0),
    //     5);
    // CANRetrier.retryStatusCode(
    //     () -> steerEncoder.getConfigurator().apply(cancoderConfig(rotationOffset), 1.0), 5);

    deviceManager.bringUp(this, "driveMotor", driveMotor, driveMotorConfig());
    deviceManager.bringUp(this, "steerMotor", steerMotor, steerMotorConfig(steerEncoder.getDeviceID()));
    deviceManager.bringUp(this, "steerEncoder", steerEncoder, cancoderConfig(rotationOffset));


    driveVoltSignal = driveMotor.getMotorVoltage();
    driveAmpSignal = driveMotor.getTorqueCurrent();

    steerVoltSignal = steerMotor.getMotorVoltage();
    steerAmpSignal = steerMotor.getTorqueCurrent();

    steerAbsoluteSignal = steerEncoder.getAbsolutePosition();
    steerAbsoluteVeloSignal = steerEncoder.getVelocity();

    odoThread.addModuleStatusSignals(
        moduleId,
        driveMotor.getPosition().clone(),
        driveMotor.getVelocity().clone(),
        steerMotor.getPosition().clone(),
        steerMotor.getVelocity().clone());

    driveMotor.optimizeBusUtilization(0.0, 1.0);
    steerMotor.optimizeBusUtilization(0.0, 1.0);
    steerEncoder.optimizeBusUtilization(0.0, 1.0);

    deviceManager.retryStatusCode(
        () -> driveMotor.setPosition(0.0, 0.1),
        "set drive motor position",
        3
    );

    driveMotorClosedReq = new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);

    BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "]");
  }

  protected TalonFXConfiguration driveMotorConfig() {
    var cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted =
        kDriveMotor.INVERT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode =
        kDriveMotor.NEUTRAL_MODE_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    cfg.Slot0.kP = kDriveMotor.kP;
    cfg.Slot0.kI = kDriveMotor.kI;
    cfg.Slot0.kD = kDriveMotor.kD;
    cfg.Slot0.kV = kDriveMotor.kV / Conv.RADIANS_TO_ROTATIONS;
    cfg.Slot0.kS = kDriveMotor.kS;

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = kDriveMotor.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = kDriveMotor.SUPPLY_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 0.3;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = kDriveMotor.STATOR_CURRENT_LIMIT;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -kDriveMotor.STATOR_CURRENT_LIMIT;

    cfg.Feedback.SensorToMechanismRatio = kDriveMotor.GEAR_RATIO;

    return cfg;
  }

  protected TalonFXConfiguration steerMotorConfig(int encoderId) {
    var cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted =
        kSteerMotor.INVERT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode =
        kSteerMotor.NEUTRAL_MODE_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    cfg.Slot0.kP = kSteerMotor.kP;
    cfg.Slot0.kI = kSteerMotor.kI;
    cfg.Slot0.kD = kSteerMotor.kD;
    cfg.Slot0.kV = kSteerMotor.kV / Conv.RADIANS_TO_ROTATIONS;

    cfg.MotionMagic.MotionMagicCruiseVelocity = kSteerMotor.MAX_VELOCITY;
    cfg.MotionMagic.MotionMagicAcceleration = kSteerMotor.MAX_ACCELERATION;

    cfg.Feedback.FeedbackRemoteSensorID = encoderId;
    cfg.Feedback.RotorToSensorRatio = kSteerMotor.GEAR_RATIO;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;

    return cfg;
  }

  protected CANcoderConfiguration cancoderConfig(double rotationOffset) {
    var canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = rotationOffset;

    return canCoderConfig;
  }

  public int getModuleId() {
    return this.moduleId;
  }

  @Override
  public void setDesiredState(AdvancedSwerveModuleState desiredState) {
    super.controlledLastCycle = true;
    desiredState.optimize(getAngle());
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
            ? lastAngle
            : desiredState.angle;
    super.targetSteerAbsoluteRads = angle.getRadians();

    steerMotor.setControl(steerMotorReq.withPosition(angle.getRotations()));
    lastAngle = angle;
  }

  private void setSpeed(AdvancedSwerveModuleState desiredState) {
    super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
    double rps = desiredState.speedMetersPerSecond / kWheel.CIRCUMFERENCE;
    log("DriveRPS", rps);
    driveMotor.setControl(
        driveMotorClosedReq.withVelocity(rps).withAcceleration(desiredState.driveAcceleration));
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(super.driveVeloMPS, getAngle());
  }

  @Override
  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(super.drivePositionMeters, getAngle());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRadians(super.steerAbsoluteRads);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !super.controlledLastCycle) {
      setVoltageOut(0.0, getAngle());
    }
    super.controlledLastCycle = false;

    super.steerAbsoluteRads = Units.rotationsToRadians(steerAbsoluteSignal.getValueAsDouble());
    super.steerVeloRadPS = Units.rotationsToRadians(steerAbsoluteVeloSignal.getValueAsDouble());
    super.steerVolts = steerVoltSignal.getValueAsDouble();
    super.steerAmps = steerAmpSignal.getValueAsDouble();

    super.drivePositionMeters = odoThread.getModulePosition(moduleId) * kWheel.CIRCUMFERENCE;
    super.driveVeloMPS = odoThread.getModuleVelocity(moduleId) * kWheel.CIRCUMFERENCE;
    super.driveVolts = driveVoltSignal.getValueAsDouble();
    super.driveAmps = driveAmpSignal.getValueAsDouble();
  }

  @Override
  public void setVoltageOut(double volts, Rotation2d angle) {
    super.controlledLastCycle = true;
    setAngle(new SwerveModuleState(0.0, angle));
    driveMotor.setVoltage(volts);
  }
}
