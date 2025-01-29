package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward.SimpleFeedforward;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class WristSim extends Wrist {
  private final ShamMCX shamMCX = new ShamMCX("WristMotor");
  private final ShamMechanism wristMechanism;

  private final ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> elevatorLoop;

  public WristSim(SimCtx simCtx) {
    elevatorLoop =
        ClosedLoop.forCurrentAngle(
            PIDFeedback.forAngular(Amps, Radians, WristConstants.KP, WristConstants.KD),
            SimpleFeedforward.forCurrent(
                Radians,
                WristConstants.KS,
                WristConstants.KV,
                WristConstants.KA,
                simCtx.robot().timing().dt()),
            UnitTrapezoidProfile.forAngle(
                RadiansPerSecond.of(WristConstants.MAX_VELOCITY),
                RadiansPerSecondPerSecond.of(WristConstants.MAX_ACCELERATION)));
    wristMechanism =
        new ShamMechanism(
            "WristMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            shamMCX,
            KilogramSquareMeters.of(.4),
            GearRatio.reduction(WristConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volt.of(0.5)),
            // MechanismDynamics.forArm(Pound.of(9.0), Inches.of(6)),
            MechanismDynamics.zero(),
            HardLimits.of(Radian.of(WristConstants.MIN_ANGLE), Radian.of(WristConstants.MAX_ANGLE)),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(wristMechanism);
    shamMCX.configSensorToMechanismRatio(WristConstants.GEAR_RATIO);
  }

  @Override
  public void goToPosition(double angleRadians) {
    super.targetRadians = angleRadians;
    shamMCX.controlCurrent(elevatorLoop, Radians.of(angleRadians));
  }

  @Override
  public double positionRadians() {
    return shamMCX.position().in(Radian);
  }

  @Override
  public void setNeutralMode(boolean shouldBeCoast) {
    shamMCX.setBrakeMode(!shouldBeCoast);
  }

  @Override
  public void periodic() {
    super.amps = shamMCX.statorCurrent().in(Amps);
    super.angleRadians = positionRadians();
    log("angleDegrees", Conv.RADIANS_TO_DEGREES * angleRadians);
    log("targetDegrees", Conv.RADIANS_TO_DEGREES * targetRadians);
  }
}
