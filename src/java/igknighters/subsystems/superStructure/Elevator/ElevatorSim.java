package igknighters.subsystems.superStructure.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import igknighters.SimCtx;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward.ElevatorFeedforward;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class ElevatorSim extends Elevator {
  private final ShamMCX shamMCX = new ShamMCX("ElevatorMotor");
  private final ShamMechanism shamMechanism;
  private final ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> elevatorLoop;

  public ElevatorSim(SimCtx simCtx) {
    shamMechanism =
        new ShamMechanism(
            "ElevatorMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(2), 2),
            shamMCX,
            KilogramSquareMeters.of(.2),
            GearRatio.reduction(ElevatorConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(2), Volts.of(0.2)),
            MechanismDynamics.forElevator(
                Pounds.of(25.0), Meters.of(ElevatorConstants.WHEEL_RADIUS * 2.0)),
            HardLimits.of(
                Rotations.of(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.WHEEL_CIRCUMFERENCE),
                Rotations.of(ElevatorConstants.MAX_HEIGHT / ElevatorConstants.WHEEL_CIRCUMFERENCE)),
            0.0,
            simCtx.robot().timing());
    shamMechanism.setState(
        new ShamMechanism.MechanismState(
            Radians.of(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.WHEEL_RADIUS),
            RadiansPerSecond.zero(),
            RadiansPerSecondPerSecond.zero()));
    simCtx.robot().addMechanism(shamMechanism);

    elevatorLoop =
        ClosedLoop.forVoltageAngle(
            PIDFeedback.forAngular(Volts, Rotations, ElevatorConstants.KP, ElevatorConstants.KD),
            ElevatorFeedforward.forAngularVoltage(
                Rotations,
                ElevatorConstants.KS,
                ElevatorConstants.KG,
                ElevatorConstants.KV,
                0.0,
                simCtx.timing().dt()),
            UnitTrapezoidProfile.forAngle(
                RotationsPerSecond.of(
                    ElevatorConstants.MAX_VELOCITY / ElevatorConstants.WHEEL_CIRCUMFERENCE),
                RotationsPerSecondPerSecond.of(
                    ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.WHEEL_CIRCUMFERENCE)));
    shamMCX.configSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO);
  }

  @Override
  public void gotoPosition(double heightMeters) {
    super.whereItsTryingToGetToInMeters = heightMeters;

    shamMCX.controlVoltage(
        elevatorLoop, Rotations.of(heightMeters / ElevatorConstants.WHEEL_CIRCUMFERENCE));
  }

  @Override
  public boolean isAtPosition(double heightMeters, double toleranceMeters) {
    super.whatIsAtIsCheckingAgainst = heightMeters;
    super.whereMotorThinksItIsInMeters =
        shamMCX.position().in(Rotations) * ElevatorConstants.WHEEL_CIRCUMFERENCE;

    return MathUtil.isNear(
        heightMeters,
        shamMCX.position().in(Rotations) * ElevatorConstants.WHEEL_CIRCUMFERENCE,
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
    super.meters = shamMCX.position().in(Rotations) * ElevatorConstants.WHEEL_CIRCUMFERENCE;
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.volts = shamMCX.voltage().in(Volts);
    super.isHomed = true;
    super.isLimitTrip = MathUtil.isNear(0.0, super.meters, 0.01);
    super.metersPerSecond =
        shamMCX.velocity().in(RadiansPerSecond) * ElevatorConstants.WHEEL_CIRCUMFERENCE;
  }
}
