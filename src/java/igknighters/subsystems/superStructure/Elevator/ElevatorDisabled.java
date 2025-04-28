package igknighters.subsystems.superStructure.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.Optional;

public class ElevatorDisabled extends Elevator {
  @Override
  public void gotoPosition(double targetPosition, Optional<Constraints> constraints) {
    super.targetMeters = targetPosition;
    super.meters = targetPosition;
    super.controlledLastCycle = true;

    var c = constraints.orElse(DEFAULT_CONSTRAINTS);
    super.maxVelocity = c.maxVelocity;
    super.maxAcceleration = c.maxAcceleration;
  }

  @Override
  public boolean home() {
    return true;
  }

  @Override
  public void setNeutralMode(boolean shouldBeCoast) {}

  @Override
  public void voltageOut(double voltage) {
    super.setNoTarget();
    super.controlledLastCycle = true;
    super.volts = voltage;
  }

  @Override
  public void periodic() {
    super.controlledLastCycle = false;
  }
}
