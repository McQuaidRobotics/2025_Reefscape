package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.SimCtx;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import java.util.Optional;
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
            GearRatio.reduction(kWrist.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volt.of(kWrist.KS * 1.3)),
            // MechanismDynamics.forArm(Pound.of(9.0), Inches.of(6)),
            MechanismDynamics.zero(),
            HardLimits.of(Radians.of(kWrist.MAX_ANGLE), Radians.of(kWrist.MIN_ANGLE)),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(wristMechanism);

    controlLoop =
        ClosedLoop.forVoltageAngle(
            PIDFeedback.forAngular(Volts, kWrist.KP, kWrist.KD),
            SimpleFeedforward.forVoltage(
                Radians, kWrist.KS, kWrist.KV, kWrist.KA, simCtx.robot().timing().dt()),
            UnitTrapezoidProfile.forAngle(
                RadiansPerSecond.of(kWrist.MAX_VELOCITY),
                RadiansPerSecondPerSecond.of(kWrist.MAX_ACCELERATION)));
    shamMCX.setBrakeMode(false); // do not enable, this feature is broken
    shamMCX.configSensorToMechanismRatio(kWrist.GEAR_RATIO);
    shamMCX.configureCurrentLimit(
        CurrentLimits.of(
            Amps.of(kWrist.STATOR_CURRENT_LIMIT), Amps.of(kWrist.SUPPLY_CURRENT_LIMIT)));
  }

  @Override
  public void goToPosition(double targetPosition, Optional<Constraints> constraints) {
    super.targetRadians = targetPosition;
    var c = constraints.orElse(DEFAULT_CONSTRAINTS);
    super.maxVelocity = c.maxVelocity;
    super.maxAcceleration = c.maxAcceleration;
    super.controlledLastCycle = true;
    shamMCX.controlVoltage(controlLoop, Radians.of(targetPosition));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    shamMCX.setBrakeMode(!coast);
  }

  @Override
  public void voltageOut(double voltage) {
    super.noTarget();
    super.controlledLastCycle = true;
    shamMCX.controlVoltage(Volts.of(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.noTarget();
      shamMCX.controlVoltage(Volts.zero());
    }
    super.controlledLastCycle = false;
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.volts = shamMCX.voltage().in(Volt);
    super.radians = shamMCX.position().in(Radians);
    super.radiansPerSecond = shamMCX.velocity().in(RadiansPerSecond);
  }
}
