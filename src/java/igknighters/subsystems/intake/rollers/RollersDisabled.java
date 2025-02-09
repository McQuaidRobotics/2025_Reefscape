package igknighters.subsystems.intake.rollers;

public class RollersDisabled extends Rollers {
  @Override
  public boolean hasAlgae() {
    return true;
  }

  @Override
  public boolean hasCoral() {
    return true;
  }

  @Override
  public void setVoltage(double voltage) {
  }

  @Override
  public void setTorque(double current) {
  }
}
