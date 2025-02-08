package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import igknighters.SimCtx;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ShamMCX;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class RollerSim extends Rollers {
  private final ShamMechanism intakMechanism;
  private final ShamMCX intakeMotor;

  public RollerSim(SimCtx simCtx) {
    intakeMotor = new ShamMCX("IntakeMotor");
    intakMechanism =
        new ShamMechanism(
            "IntakeMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            intakeMotor,
            KilogramSquareMeters.of(0.1),
            GearRatio.reduction(RollerConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volts.of(1)),
            MechanismDynamics.zero(),
            HardLimits.unbounded(),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(intakMechanism);
  }

  public void setVoltage(double voltage) {
    intakeMotor.controlVoltage(Volts.of(voltage));
  }

  @Override
  public boolean hasAlgae() {
    return true;
  }

  @Override
  public boolean hasCoral() {
    return true;
  }

  @Override
  public void periodic() {
    amps = intakeMotor.statorCurrent().in(Amps);
    rads = intakeMotor.velocity().in(RPM);
    hasAlgae = hasAlgae();
    hasCoral = hasCoral();
  }
}
