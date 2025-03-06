package igknighters.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import igknighters.subsystems.intake.IntakeConstants.RollerConstants;

public class RollersDisabled extends Rollers {
  public RollersDisabled() {
    super(DCMotor.getKrakenX60(1).withReduction(RollerConstants.GEAR_RATIO));
  }

  @Override
  public void voltageOut(double voltage) {}

  @Override
  public void currentOut(double current) {}

  @Override
  public boolean isStalling() {
    return false;
  }
}
