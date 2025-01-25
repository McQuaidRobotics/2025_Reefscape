package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.system.plant.DCMotor;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ShamMCX;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class WristSim {
  private final ShamMCX shamMCX = new ShamMCX("WristMotor");
  private final ShamMechanism WristMechanism;

  public WristSim(SimCtx simCtx) {
    WristMechanism =
        new ShamMechanism(
            "WristMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            shamMCX,
            KilogramSquareMeters.of(.2),
            GearRatio.reduction(6.0),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volt.of(1)),
            MechanismDynamics.forArm(Pound.of(9.0), Inches.of(6)),
            HardLimits.of(
                Radian.of(Conv.DEGREES_TO_RADIANS * WristConstants.MIN_ANGLE),
                Radian.of(Conv.DEGREES_TO_RADIANS * WristConstants.MAX_ANGLE)),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(WristMechanism);
  }

  public void setPower(double power) {
    // TODO Auto-generated method stub

  }

  public void setNeutralMode(boolean shouldBeCoast) {
    // TODO Auto-generated method stub

  }

  public double positionDegrees() {
    // TODO Auto-generated method stub
    return 0;
  }
}
