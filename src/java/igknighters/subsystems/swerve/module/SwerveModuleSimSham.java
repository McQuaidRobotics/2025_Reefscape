package igknighters.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import sham.ShamSwerve;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward.SimpleFeedforward;
import wayfinder.setpointGenerator.AdvancedSwerveModuleState;

public class SwerveModuleSimSham extends SwerveModule {

  private boolean gotDirectionsLastCycle = false;

  private final int moduleId;

  private final ShamMCX driveMotor;
  private final ShamMCX steerMotor;
  private final ClosedLoop<VoltageUnit, AngularVelocityUnit> driveLoop;
  private final ClosedLoop<VoltageUnit, AngleUnit> steerLoop;

  public SwerveModuleSimSham(
      final int moduleId, SimSwerveOdometryThread odoThread, ShamSwerve sim) {
    super("SwerveModule[" + moduleId + "]");
    this.moduleId = moduleId;

    driveMotor = new ShamMCX("DriveMotor[" + moduleId + "]");
    steerMotor = new ShamMCX("SteerMotor[" + moduleId + "]");

    odoThread.addModulePositionSupplier(moduleId, this::getCurrentPosition);

    driveLoop =
        ClosedLoop.forVoltageAngularVelocity(
            PIDFeedback.forAngularVelocity(Volts, kDriveMotor.kP),
            SimpleFeedforward.forVoltage(
                Rotations, kDriveMotor.kS, kDriveMotor.kV, 0.0, sim.timing().dt()));

    steerLoop =
        ClosedLoop.forVoltageAngle(
            PIDFeedback.forAngular(Volts, kSteerMotor.kP, kSteerMotor.kD)
                .withContinuousAngularInput(),
            SimpleFeedforward.forVoltage(Rotations, kSteerMotor.kS, 0.0, 0.0, sim.timing().dt()));

    steerMotor.configSensorToMechanismRatio(kSwerve.STEER_GEAR_RATIO);

    sim.withSetModuleControllers(moduleId, driveMotor, steerMotor);

    BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "] initialized (sim)");
  }

  public void setDesiredState(AdvancedSwerveModuleState desiredState) {
    gotDirectionsLastCycle = true;
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(
        driveMotor.position().div(kSwerve.DRIVE_GEAR_RATIO).in(Rotations)
            * kSwerve.WHEEL_CIRCUMFERENCE,
        new Rotation2d(steerMotor.position()));
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        driveMotor.velocity().div(kSwerve.DRIVE_GEAR_RATIO).in(RotationsPerSecond)
            * kSwerve.WHEEL_CIRCUMFERENCE,
        new Rotation2d(steerMotor.position()));
  }

  @Override
  public int getModuleId() {
    return this.moduleId;
  }

  private void setAngle(AdvancedSwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
            ? new Rotation2d(super.steerAbsoluteRads)
            : desiredState.angle;
    super.targetSteerAbsoluteRads = angle.getRadians();
    steerMotor.controlVoltage(
        steerLoop, angle.getMeasure(), RadiansPerSecond.of(desiredState.steerVelocity));
  }

  private void setSpeed(AdvancedSwerveModuleState desiredState) {
    super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
    driveMotor.controlVoltage(
        driveLoop,
        RotationsPerSecond.of(
            (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE)
                * kSwerve.DRIVE_GEAR_RATIO),
        RadiansPerSecondPerSecond.of(desiredState.driveAcceleration));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !gotDirectionsLastCycle) {
      targetDriveVeloMPS = 0.0;
    }
    log("gotDirectionsLastCycle", gotDirectionsLastCycle);
    gotDirectionsLastCycle = false;

    super.drivePositionMeters =
        driveMotor.position().div(kSwerve.DRIVE_GEAR_RATIO).in(Rotations)
            * kSwerve.WHEEL_CIRCUMFERENCE;
    super.driveVeloMPS =
        driveMotor.velocity().div(kSwerve.DRIVE_GEAR_RATIO).in(RotationsPerSecond)
            * kSwerve.WHEEL_CIRCUMFERENCE;

    super.steerAbsoluteRads = MathUtil.angleModulus(steerMotor.position().in(Radians));
    super.steerVeloRadPS = steerMotor.velocity().in(RadiansPerSecond);

    super.driveVolts = driveMotor.voltage().in(Volts);
    super.steerVolts = steerMotor.voltage().in(Volts);
    super.driveAmps = driveMotor.statorCurrent().in(Amps);
    super.steerAmps = steerMotor.statorCurrent().in(Amps);
  }

  @Override
  public void setVoltageOut(double volts, Rotation2d angle) {
    super.targetSteerAbsoluteRads = angle.getRadians();
    driveMotor.controlVoltage(Volts.of(volts));
    steerMotor.controlVoltage(steerLoop, angle.getMeasure());
  }
}
