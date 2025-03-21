package igknighters.subsystems.superStructure.Wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.Optional;

public class WristDisabled extends Wrist {

  @Override
  public void goToPosition(double targetPosition, Optional<Constraints> constraints) {
    super.radians = targetPosition;
    super.targetRadians = targetPosition;
    super.controlledLastCycle = true;

    var c = constraints.orElse(DEFAULT_CONSTRAINTS);
    super.maxVelocity = c.maxVelocity;
    super.maxAcceleration = c.maxAcceleration;
  }

  @Override
  public void setNeutralMode(boolean coast) {}

  @Override
  public void voltageOut(double voltage) {
    super.noTarget();
    super.controlledLastCycle = true;
    super.volts = voltage;
  }

  @Override
  public void periodic() {
    super.controlledLastCycle = false;
  }
}
