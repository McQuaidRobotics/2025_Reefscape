package igknighters.subsystems.intake;

import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.intake.rollers.RollerSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersReal;

public class Intake implements ExclusiveSubsystem {
  private final Rollers rollers;

  public Intake(SimCtx simCtx) {
    if (Robot.isReal()) {
      rollers = new RollersReal();
    } else {
      rollers = new RollerSim(simCtx);
    }
  }
  public void runAtVoltage(double volts){
    rollers.setVoltage(volts);
  }
  public void periodic() {
    rollers.periodic();
  }
}
