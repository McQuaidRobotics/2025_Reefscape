package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

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
import sham.shamController.ShamMCX.CurrentLimits;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward.SimpleFeedforward;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class WristSim extends Wrist {
  private final ShamMCX shamMCX = new ShamMCX("WristMotor");
  private final ShamMechanism wristMechanism;

  private final ClosedLoop<VoltageUnit, AngleUnit> controlLoop;

  public WristSim(SimCtx simCtx) {
    wristMechanism =
        new ShamMechanism(
            "WristMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            shamMCX,
            KilogramSquareMeters.of(.3),
            GearRatio.reduction(WristConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volt.of(WristConstants.KS)),
            // MechanismDynamics.forArm(Pound.of(9.0), Inches.of(6)),
            MechanismDynamics.zero(),
            HardLimits.of(
                Radian.of(WristConstants.REVERSE_LIMIT),
                Radian.of(WristConstants.FORWARD_LIMIT + 0.02)),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(wristMechanism);

    controlLoop =
        ClosedLoop.forVoltageAngle(
            PIDFeedback.forAngular(Volts, Radians, WristConstants.KP, WristConstants.KD),
            SimpleFeedforward.forVoltage(
                Radians,
                WristConstants.KS,
                WristConstants.KV,
                WristConstants.KA,
                simCtx.robot().timing().dt()),
            UnitTrapezoidProfile.forAngle(
                RadiansPerSecond.of(WristConstants.MAX_VELOCITY),
                RadiansPerSecondPerSecond.of(WristConstants.MAX_ACCELERATION)));
    shamMCX.setBrakeMode(true);
    shamMCX.configSensorToMechanismRatio(WristConstants.GEAR_RATIO);
    shamMCX.configureCurrentLimit(CurrentLimits.of(Amps.of(WristConstants.STATOR_CURRENT_LIMIT), Amps.of(WristConstants.SUPPLY_CURRENT_LIMIT)));
  }

  @Override
  public void goToPosition(double targetPosition) {
    super.targetRadians = targetPosition;
    shamMCX.controlVoltage(controlLoop, Radians.of(targetPosition));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    shamMCX.setBrakeMode(!coast);
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetRadians = Double.NaN;
    shamMCX.controlVoltage(Volts.of(voltage));
  }

  @Override
  public void periodic() {
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.volts = shamMCX.voltage().in(Volt);
    super.radians = shamMCX.position().in(Radians);
    super.radiansPerSecond = shamMCX.velocity().in(RadiansPerSecond);
  }
}
