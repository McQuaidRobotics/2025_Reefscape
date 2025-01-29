package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward.ArmFeedforward;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class WristSim extends Wrist {
  private final ShamMCX shamMCX = new ShamMCX("WristMotor");
  private final ShamMechanism WristMechanism;

  private final ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> elevatorLoop;

  public WristSim(SimCtx simCtx) {
    elevatorLoop =
        ClosedLoop.forCurrentAngle(
            PIDFeedback.forAngular(Amps, Radians, WristConstants.KP, WristConstants.KD),
            ArmFeedforward.forCurrent(
                Radians,
                WristConstants.KS,
                WristConstants.KG,
                WristConstants.KV,
                WristConstants.KA,
                simCtx.robot().timing().dt()),
            true);
    WristMechanism =
        new ShamMechanism(
            "WristMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            shamMCX,
            KilogramSquareMeters.of(.2),
            GearRatio.reduction(ElevatorConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volt.of(1)),
            MechanismDynamics.forArm(Pound.of(9.0), Inches.of(6)),
            HardLimits.of(
                Radian.of(Conv.DEGREES_TO_RADIANS * WristConstants.MIN_ANGLE),
                Radian.of(Conv.DEGREES_TO_RADIANS * WristConstants.MAX_ANGLE)),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(WristMechanism);
    shamMCX.configSensorToMechanismRatio(WristConstants.GEAR_RATIO);
  }

  @Override
  public void goToPosition(double angleDegrees) {
    super.targetingDegrees = angleDegrees;
    shamMCX.controlCurrent(elevatorLoop, Radians.of(angleDegrees));
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
    super.angleDegrees = positionRadians() * Conv.RADIANS_TO_DEGREES;
  }
}
