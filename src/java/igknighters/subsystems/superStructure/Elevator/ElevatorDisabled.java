package igknighters.subsystems.superStructure.Elevator;

public class ElevatorDisabled extends Elevator {
  @Override
  public void gotoPosition(double targetPosition) {
    super.targetMeters = targetPosition;
    super.meters = targetPosition;
    super.controlledLastCycle = true;
  }

  @Override
  public boolean home() {
    return true;
  }

  @Override
  public void setNeutralMode(boolean shouldBeCoast) {}

  @Override
  public void voltageOut(double voltage) {
    super.targetMeters = Double.NaN;
    super.controlledLastCycle = true;
    super.volts = voltage;
  }

  @Override
  public void periodic() {
    super.controlledLastCycle = false;
  }
}
