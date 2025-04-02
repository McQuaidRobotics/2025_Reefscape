package igknighters.subsystems.intake.rollers;

public class RollersDisabled extends Rollers {
  @Override
  public void voltageOut(double voltage) {}

  @Override
  public void currentOut(double current) {}

  @Override
  public boolean isStalling() {
    return false;
  }

  @Override
  public double getVoltage() {
    return 0;
  }
}
