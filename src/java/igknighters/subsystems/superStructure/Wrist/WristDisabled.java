package igknighters.subsystems.superStructure.Wrist;

public class WristDisabled extends Wrist {
  @Override
  public void goToPosition(double targetPosition) {
    super.radians = targetPosition;
    super.targetRadians = targetPosition;
  }

  @Override
  public void setNeutralMode(boolean coast) {}

  @Override
  public void voltageOut(double voltage) {
    super.targetRadians = Double.NaN;
    super.volts = voltage;
  }
}
