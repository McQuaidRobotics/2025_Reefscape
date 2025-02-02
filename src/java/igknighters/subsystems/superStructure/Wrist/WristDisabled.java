package igknighters.subsystems.superStructure.Wrist;

public class WristDisabled extends Wrist {
  @Override
  public void goToPosition(double targetPosition) {
    super.radians = targetPosition;
    super.targetRadians = targetPosition;
    super.controlledLastCycle = true;
  }

  @Override
  public void setNeutralMode(boolean coast) {}

  @Override
  public void voltageOut(double voltage) {
    super.targetRadians = Double.NaN;
    super.controlledLastCycle = true;
    super.volts = voltage;
  }

  @Override
  public void periodic() {
    super.controlledLastCycle = false;
  }
}
