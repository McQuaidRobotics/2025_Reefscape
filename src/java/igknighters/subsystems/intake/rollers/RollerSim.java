package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
  private final ShamMechanism intakeMechanism;
  private final ShamMCX intakeMotor;

  public RollerSim(SimCtx simCtx) {
    intakeMotor = new ShamMCX("IntakeMotor");
    intakeMechanism =
        new ShamMechanism(
            "IntakeMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            intakeMotor,
            KilogramSquareMeters.of(0.03),
            GearRatio.reduction(RollerConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volts.of(0.12)),
            MechanismDynamics.zero(),
            HardLimits.unbounded(),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(intakeMechanism);
  }

  public void setVoltage(double voltage) {
    intakeMotor.controlVoltage(Volts.of(voltage));
  }

  public void setTorque(double current) {
    intakeMotor.controlCurrent(Amps.of(current));
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
    super.volts = intakeMotor.voltage().in(Volts);
    super.current = intakeMotor.statorCurrent().in(Amps);
    super.radiansPerSecond = intakeMotor.velocity().in(RadiansPerSecond);
    super.hasAlgae = hasAlgae();
    super.hasCoral = hasCoral();
  }
}
