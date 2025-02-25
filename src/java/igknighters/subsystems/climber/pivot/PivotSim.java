package igknighters.subsystems.climber.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.climber.ClimberConstants.PivotConstants;
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

public class PivotSim extends Pivot {
  private final ShamMCX shamMCX = new ShamMCX("WristMotor");
  private final ShamMechanism wristMechanism;

  private final ClosedLoop<VoltageUnit, AngleUnit> controlLoop;

  public PivotSim(SimCtx simCtx) {
    wristMechanism =
        new ShamMechanism(
            "WristMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            shamMCX,
            KilogramSquareMeters.of(.3),
            GearRatio.reduction(PivotConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volt.of(1.0)),
            // MechanismDynamics.forArm(Pound.of(9.0), Inches.of(6)),
            MechanismDynamics.zero(),
            HardLimits.of(
                Rotations.of(PivotConstants.REVERSE_LIMIT),
                Rotations.of(PivotConstants.FORWARD_LIMIT + 0.02)),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(wristMechanism);

    controlLoop =
        ClosedLoop.forVoltageAngle(
            PIDFeedback.forAngular(Volts, PivotConstants.KP, PivotConstants.KD),
            SimpleFeedforward.forVoltage(Rotations, 0.0, 0.0, 0.0, simCtx.robot().timing().dt()),
            UnitTrapezoidProfile.forAngle(
                RotationsPerSecond.of(PivotConstants.MAX_VELOCITY),
                RotationsPerSecondPerSecond.of(PivotConstants.MAX_ACCELERATION)));
    shamMCX.setBrakeMode(false); // do not enable, this feature is broken
    shamMCX.configSensorToMechanismRatio(PivotConstants.GEAR_RATIO);
    shamMCX.configureCurrentLimit(
        CurrentLimits.of(
            Amps.of(PivotConstants.STATOR_CURRENT_LIMIT),
            Amps.of(PivotConstants.SUPPLY_CURRENT_LIMIT)));
  }

  @Override
  public void setPositionRads(double targetRads) {
    super.targetRads = targetRads;
    super.controlledLastCycle = true;
    shamMCX.controlVoltage(controlLoop, Radians.of(targetRads));
  }

  public boolean isAtPosition(double positionRads, double toleranceRads) {
    return MathUtil.isNear(
        positionRads, shamMCX.position().in(Rotations) * Conv.ROTATIONS_TO_RADIANS, toleranceRads);
  }

  public double getPositionRads() {
    return shamMCX.position().in(Radians);
  }

  @Override
  public void setNeutralMode(boolean coast) {
    shamMCX.setBrakeMode(!coast);
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetRads = Double.NaN;
    super.controlledLastCycle = true;
    shamMCX.controlVoltage(Volts.of(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetRads = Double.NaN;
      shamMCX.controlVoltage(Volts.zero());
    }
    super.controlledLastCycle = false;
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.radians = shamMCX.position().in(Radians);
  }
}
