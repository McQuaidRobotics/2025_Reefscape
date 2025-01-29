package igknighters.subsystems.superStructure.Elevator;

public class ElevatorDisabled extends Elevator {
  @Override
  public void gotoPosition(double height) {}

  @Override
  public boolean isAtPosition(double position, double tolerance) {
    return false;
  }

  @Override
  public boolean home() {
    return true;
  }

  @Override
  public void setNeutralMode(boolean shoodBeCoast) {}
}
