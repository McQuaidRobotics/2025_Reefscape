package igknighters.subsystems.intake;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.SharedState;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.intake.rollers.RollerSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersReal;
import monologue.Annotations.Log;
import monologue.ProceduralStructGenerator;

public class Intake implements ExclusiveSubsystem {
  private final SharedState shared;

  @Log private Holding currentlyHolding = Holding.NONE;
  @Log private Holding tryingToHold = Holding.NONE;

  public enum Holding implements StructSerializable {
    CORAL,
    ALGAE,
    NONE;

    public static final Struct<Holding> struct = ProceduralStructGenerator.genEnum(Holding.class);
  }

  public enum ControlType {
    VOLTAGE,
    CURRENT,
    TORQUE,
    VELOCITY
  }

  private final Rollers rollers;

  public Intake(SharedState shared, SimCtx simCtx) {
    this.shared = shared;
    if (Robot.isReal()) {
      rollers = new RollersReal();
    } else {
      rollers = new RollerSim(simCtx);
    }
  }

  public void control(ControlType controlType, double value) {
    if (value > -0.01) {
      currentlyHolding = Holding.NONE;
    }
    switch (controlType) {
      case VOLTAGE -> rollers.voltageOut(value);
      case CURRENT -> rollers.currentOut(value);
      case TORQUE -> rollers.torqueOut(value);
      case VELOCITY -> rollers.velocityOut(value);
    }
  }

  public Holding getHolding() {
    return currentlyHolding;
  }

  public void setTryingToHold(Holding holding) {
    tryingToHold = holding;
  }

  public double gamepieceYOffset() {
    return rollers.gamepieceDistance();
  }

  public void periodic() {
    rollers.periodic();

    if (tryingToHold != Holding.NONE && rollers.isLaserTripped()) {
      currentlyHolding = tryingToHold;
      tryingToHold = Holding.NONE;
    } else if (tryingToHold == Holding.NONE
        && currentlyHolding == Holding.NONE
        && rollers.isLaserTripped()) {
      currentlyHolding = Holding.CORAL;
    } else if (!rollers.isLaserTripped()) {
      currentlyHolding = Holding.NONE;
    }
    shared.holdingAlgae = getHolding() == Holding.ALGAE;
  }
}
