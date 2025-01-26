package igknighters.subsystems.Intake;

import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Intake.rollers.Rollers;
import igknighters.subsystems.Intake.rollers.RollersReal;
import igknighters.subsystems.Intake.rollers.RollersSim;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;

public class Intake implements ExclusiveSubsystem {
  private final Rollers rollers;

  public Intake(SimCtx simCtx) {
    if (Robot.isReal()) {
      rollers = new RollersReal();
    } else {
      rollers = new RollersSim(simCtx);
    }
  }

  public void setCurrent(double amps) {
    rollers.setCurrent(amps);
  }

  public boolean hasCoral() {
    return rollers.hasCoral();
  }

  public boolean hasAlgae() {
    return rollers.hasAlgae();
  }
}
