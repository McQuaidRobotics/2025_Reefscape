package igknighters.subsystems.intake;

import java.util.function.BooleanSupplier;

import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.intake.rollers.RollerSim;
import igknighters.subsystems.intake.rollers.Rollers;
import igknighters.subsystems.intake.rollers.RollersReal;

public class Intake implements ExclusiveSubsystem {
  public enum Holding {
    CORAL,
    ALGAE,
    NONE
  }

  private final Rollers rollers;

  public Intake(SimCtx simCtx) {
    if (Robot.isReal()) {
      rollers = new RollersReal();
    } else {
      rollers = new RollerSim(simCtx);
    }
  }

  public void runAtVoltage(double volts){
    rollers.setVoltage(volts);
  }

  public void runAtTorque(double torque){
    rollers.setTorque(torque);
  }

  public Holding getHolding(){
    if(rollers.hasCoral()){
      return Holding.CORAL;
    } else if(rollers.hasAlgae()){
      return Holding.ALGAE;
    } else {
      return Holding.NONE;
    }
  }

  public BooleanSupplier isHolding(Holding holding){
    return () -> getHolding() == holding;
  }

  public void periodic() {
    rollers.periodic();
  }
}
