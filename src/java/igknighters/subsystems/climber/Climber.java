package igknighters.subsystems.climber;

import igknighters.DeviceManager;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.climber.pivot.Pivot;
import igknighters.subsystems.climber.pivot.PivotReal;
import igknighters.subsystems.climber.pivot.PivotSim;

public class Climber implements ExclusiveSubsystem {
  private final Pivot pivot;

  public Climber(DeviceManager deviceManager, SimCtx simCtx) {
    if (Robot.isReal()) {
      pivot = new PivotReal(deviceManager);
    } else {
      pivot = new PivotSim(simCtx);
    }
  }

  public void setPivotPosition(double position) {
    pivot.setPositionRads(position);
  }

  public boolean isPivotAtPosition(double position, double tolerance) {
    return pivot.isAtPosition(position, tolerance);
  }

  public void voltageOut(double voltage) {
    pivot.voltageOut(voltage);
  }

  @Override
  public void periodic() {
    pivot.periodic();
  }
}
