package igknighters.subsystems.superStructure;

import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.Robot;
import igknighters.subsystems.Intake.Intake;
import igknighters.subsystems.Intake.IntakeReal;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorReal;
import igknighters.subsystems.superStructure.Wrist.Wrist;
import igknighters.subsystems.superStructure.Wrist.WristReal;

public class SuperStructure implements Subsystem {
  private final Wrist wrist;
  private final Elevator elevator;

  public SuperStructure() {
      if (Robot.isReal()) {
          wrist = new WristReal();
          elevator = new ElevatorReal();
      } else {
          wrist = null;
          elevator = null;
          throw new RuntimeException("Sim not supported yet");
      }
  }

  public void gotoPosition(double elevatorHeight, double wristAngle) {
    elevator.gotoPosition(elevatorHeight);
    wrist.goToPosition(wristAngle);
  }

  public boolean superStructureIsAt(double height, double wristAngle, double elevatorTolerance, double wristTolerance) {
    return elevator.isAtPosition(height, elevatorTolerance) && wrist.isAtPosition(wristAngle, wristTolerance);
  }
}

