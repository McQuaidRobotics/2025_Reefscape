package igknighters.subsystems.superStructure;

import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorReal;
import igknighters.subsystems.superStructure.Elevator.ElevatorSim;
import igknighters.subsystems.superStructure.Wrist.Wrist;
import igknighters.subsystems.superStructure.Wrist.WristReal;
import igknighters.subsystems.superStructure.Wrist.WristSim;

public class SuperStructure implements ExclusiveSubsystem {
  private final SuperStructureVisualizer visualizer;
  private final Wrist wrist;
  private final Elevator elevator;

  public SuperStructure(SimCtx simCtx) {
    visualizer = new SuperStructureVisualizer();
    if (Robot.isReal()) {
      wrist = new WristReal();
      elevator = new ElevatorReal();
    } else {
      wrist = new WristSim(simCtx);
      elevator = new ElevatorSim(simCtx);
    }
  }

  public void goTo(double elevatorMeters, double wristRads) {
    elevator.gotoPosition(elevatorMeters);
    wrist.goToPosition(wristRads);
    visualizer.updateSetpoint(elevatorMeters, wristRads);
  }

  public boolean isAt(
      double height, double wristAngle, double elevatorTolerance, double wristTolerance) {
    return (elevator.isAtPosition(height, elevatorTolerance)
        && wrist.isAtPosition(wristAngle, wristTolerance));
  }

  @Override
  public void periodic() {
    elevator.periodic();
    wrist.periodic();
    visualizer.updatePosition(elevator.position(), wrist.position());
  }
}
