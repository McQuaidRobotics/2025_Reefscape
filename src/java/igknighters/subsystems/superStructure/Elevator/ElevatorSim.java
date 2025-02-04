package igknighters.subsystems.superStructure.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.SimCtx;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.ShamMCX.CurrentLimits;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward.ElevatorFeedforward;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class ElevatorSim extends Elevator {
  private final ShamMCX shamMCX = new ShamMCX("ElevatorMotor");
  private final ShamMechanism shamMechanism;
  private final ClosedLoop<VoltageUnit, AngleUnit> voltageLoop;

  public ElevatorSim(SimCtx simCtx) {
    shamMechanism =
        new ShamMechanism(
            "ElevatorMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(2), 2),
            shamMCX,
            KilogramSquareMeters.of(ElevatorConstants.MOI),
            GearRatio.reduction(ElevatorConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(2), Volts.of(ElevatorConstants.KS)),
            // MechanismDynamics.forElevator(
            //     Pounds.of(22.0), Meters.of(ElevatorConstants.PULLEY_RADIUS * 2.0)),
            MechanismDynamics.zero(),
            HardLimits.of(
                Radians.of(ElevatorConstants.REVERSE_LIMIT),
                Radians.of(ElevatorConstants.FORWARD_LIMIT)),
            0.0,
            simCtx.robot().timing());
    shamMechanism.setState(
        new ShamMechanism.MechanismState(
            Radians.of(ElevatorConstants.REVERSE_LIMIT),
            RadiansPerSecond.zero(),
            RadiansPerSecondPerSecond.zero()));
    simCtx.robot().addMechanism(shamMechanism);

    voltageLoop =
        ClosedLoop.forVoltageAngle(
            PIDFeedback.forAngular(Volts, Rotations, ElevatorConstants.KP, ElevatorConstants.KD),
            ElevatorFeedforward.forVoltage(
                Radians,
                ElevatorConstants.KS,
                ElevatorConstants.KG,
                ElevatorConstants.KV,
                ElevatorConstants.KA,
                simCtx.timing().dt()),
            UnitTrapezoidProfile.forAngle(
                RadiansPerSecond.of(ElevatorConstants.MAX_VELOCITY),
                RadiansPerSecondPerSecond.of(ElevatorConstants.MAX_ACCELERATION)));
    shamMCX.setBrakeMode(false);
    shamMCX.configSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO);
    shamMCX.configureCurrentLimit(
        CurrentLimits.of(
            Amps.of(ElevatorConstants.STATOR_CURRENT_LIMIT),
            Amps.of(ElevatorConstants.SUPPLY_CURRENT_LIMIT)));
  }

  @Override
  public void gotoPosition(double targetPosition) {
    super.targetMeters = targetPosition;
    super.controlledLastCycle = true;
    shamMCX.controlVoltage(
        voltageLoop, Rotations.of(targetPosition / ElevatorConstants.PULLEY_CIRCUMFERENCE));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    shamMCX.setBrakeMode(!coast);
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetMeters = Double.NaN;
    super.controlledLastCycle = true;
    shamMCX.controlVoltage(Volts.of(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetMeters = Double.NaN;
      shamMCX.controlVoltage(Volts.zero());
    }
    super.controlledLastCycle = false;
    super.meters = shamMCX.position().in(Rotations) * ElevatorConstants.PULLEY_CIRCUMFERENCE;
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.volts = shamMCX.voltage().in(Volts);
    super.isHomed = true;
    super.isLimitTripped = MathUtil.isNear(0.0, super.meters, 0.015);
    super.metersPerSecond =
        shamMCX.velocity().in(RotationsPerSecond) * ElevatorConstants.PULLEY_CIRCUMFERENCE;
  }
}
