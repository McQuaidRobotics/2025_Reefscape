package igknighters.subsystems.Intake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
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

@SuppressWarnings("unused")
public class RollersSim extends Rollers {
  private final ShamMCX shamMCX = new ShamMCX("IntakeMotor");
  private final ShamMechanism shamMechanism;

  // private final ShamIntake shamIntake;
  // private final ShamIndexer indexer;

  public RollersSim(SimCtx simCtx) {
    shamMechanism =
        new ShamMechanism(
            "IntakeMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            shamMCX,
            KilogramSquareMeters.of(0.07),
            GearRatio.reduction(2.0),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volts.of(0.5)),
            MechanismDynamics.zero(),
            HardLimits.unbounded(),
            0.0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(shamMechanism);
    // shamIntake =
    //     simCtx.robot().createIntake(new Rectangle2d(new Translation2d(), new Translation2d()));
    // indexer = simCtx.robot().getIndexer();
  }

  @Override
  public boolean hasAlgae() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public boolean hasCoral() {
    return false;
  }

  @Override
  public void setCurrent(double current) {
    shamMCX.controlCurrent(Amps.of(current));
  }
}
