package igknighters.subsystems.intake;

import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.intake.rollers.RollerSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersReal;
import java.util.function.BooleanSupplier;

public class Intake implements ExclusiveSubsystem {
  public enum Holding {
    CORAL,
    ALGAE,
    NONE
  }

  public enum ControlType {
    VOLTAGE,
    CURRENT,
    TORQUE,
    VELOCITY
  }

  private final Rollers rollers;

  public Intake(SimCtx simCtx) {
    if (Robot.isReal()) {
      rollers = new RollersReal();
    } else {
      rollers = new RollerSim(simCtx);
    }
  }

  public void control(ControlType controlType, double value) {
    switch (controlType) {
      case VOLTAGE -> rollers.voltageOut(value);
      case CURRENT -> rollers.currentOut(value);
      case TORQUE -> rollers.torqueOut(value);
      case VELOCITY -> rollers.velocityOut(value);
    }
  }

  public Holding getHolding() {
    if (rollers.hasCoral()) {
      return Holding.CORAL;
    } else if (rollers.hasAlgae()) {
      return Holding.ALGAE;
    } else {
      return Holding.NONE;
    }
  }

  public BooleanSupplier isHolding(Holding holding) {
    return () -> getHolding() == holding;
  }

  public void periodic() {
    rollers.periodic();
  }
}
