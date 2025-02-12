package igknighters.subsystems.climber;

import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.climber.electroMagnet.ElectroMagnet;
import igknighters.subsystems.climber.electroMagnet.ElectroMagnetReal;
import igknighters.subsystems.climber.electroMagnet.ElectroMagnetSim;
import igknighters.subsystems.climber.pivot.Pivot;
import igknighters.subsystems.climber.pivot.PivotReal;
import igknighters.subsystems.climber.pivot.PivotSim;

public class Climber implements ExclusiveSubsystem {
  private final ElectroMagnet magnet;
  private final Pivot pivot;

  public Climber(SimCtx simCtx) {
    if (Robot.isReal()) {
      pivot = new PivotReal();
      magnet = new ElectroMagnetReal();

    } else {
      pivot = new PivotSim(simCtx);
      magnet = new ElectroMagnetSim();
    }
  }

  public void enableMagnet(boolean enable) {
    magnet.setOn(enable);
  }

  public void setPivotPosition(double position) {
    pivot.setPositionRads(position);
  }

  @Override
  public void periodic() {
    pivot.periodic();
  }
}
