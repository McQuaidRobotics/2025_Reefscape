package igknighters.subsystems.superStructure.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import igknighters.SimCtx;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.UnitSafeControl.AngularPIDFeedback;
import sham.shamController.UnitSafeControl.FlywheelFeedforward;
import sham.shamController.UnitSafeControl.TrapezoidProfile;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class ElevatorSim extends Elevator {
  private final ShamMCX shamMCX = new ShamMCX("ElevatorMotor");
  private final ShamMechanism shamMechanism;
  private final ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> elevatorLoop;

  public ElevatorSim(SimCtx simCtx) {
    shamMechanism =
        new ShamMechanism(
            "ElevatorMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(2), 2),
            shamMCX,
            KilogramSquareMeters.of(.2),
            GearRatio.reduction(ElevatorConstants.GEAR_RATIO),
            // Friction.of(DCMotor.getKrakenX60Foc(2), Volts.of(0.25)),
            // MechanismDynamics.forElevator(Pounds.of(35.0),
            // Meters.of(ElevatorConstants.WHEEL_RADIUS * 2.0)),
            Friction.zero(),
            MechanismDynamics.zero(),
            HardLimits.of(
                Rotations.of(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.WHEEL_CIRCUMFERENCE),
                Rotations.of(ElevatorConstants.MAX_HEIGHT / ElevatorConstants.WHEEL_CIRCUMFERENCE)),
            0.0,
            simCtx.robot().timing());

    simCtx.robot().addMechanism(shamMechanism);
    elevatorLoop =
        ClosedLoop.forCurrentAngle(
            new AngularPIDFeedback<CurrentUnit>(
                Amps.per(Rotation).ofNative(ElevatorConstants.KP),
                Amps.per(RotationsPerSecond).ofNative(ElevatorConstants.KD)),
            new FlywheelFeedforward<CurrentUnit>(Amps.of(ElevatorConstants.KS)),
            TrapezoidProfile.forAngle(
                RotationsPerSecond.of(
                    ElevatorConstants.MAX_VELOCITY / ElevatorConstants.WHEEL_CIRCUMFERENCE),
                RotationsPerSecondPerSecond.of(
                    ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.WHEEL_CIRCUMFERENCE)));
    shamMCX.configSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO);
  }

  @Override
  public void gotoPosition(double heightMeters) {
    shamMCX.controlCurrent(elevatorLoop, Radians.of(heightMeters));
  }

  @Override
  public boolean isAtPosition(double heightMeters, double toleranceMeters) {
    return MathUtil.isNear(
        heightMeters / ElevatorConstants.WHEEL_RADIUS,
        shamMCX.position().in(Radians),
        toleranceMeters);
  }

  @Override
  public void setNeutralMode(boolean shouldBeCoast) {
    shamMCX.setBrakeMode(!shouldBeCoast);
  }

  @Override
  public boolean home() {
    return true;
  }

  @Override
  public void periodic() {
    super.meters = shamMCX.position().in(Radians) * ElevatorConstants.WHEEL_RADIUS;
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.volts = shamMCX.voltage().in(Volts);
    super.isHomed = true;
    super.isLimitTrip = MathUtil.isNear(0.0, super.meters, 0.01);
    super.metersPerSecond =
        shamMCX.velocity().in(RadiansPerSecond) * ElevatorConstants.WHEEL_RADIUS;
  }
}
