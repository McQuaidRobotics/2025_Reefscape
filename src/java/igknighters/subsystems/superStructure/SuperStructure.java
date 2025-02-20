package igknighters.subsystems.superStructure;

import edu.wpi.first.math.MathUtil;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorReal;
import igknighters.subsystems.superStructure.Elevator.ElevatorSim;
import igknighters.subsystems.superStructure.SuperStructureConstants.ElevatorConstants;
import igknighters.subsystems.superStructure.SuperStructureConstants.WristConstants;
import igknighters.subsystems.superStructure.Wrist.Wrist;
import igknighters.subsystems.superStructure.Wrist.WristDisabled;
import igknighters.subsystems.superStructure.Wrist.WristSim;
import java.util.OptionalDouble;

public class SuperStructure implements ExclusiveSubsystem {
  private final SuperStructureVisualizer visualizer;
  private final Wrist wrist;
  private final Elevator elevator;

  /**
   * Checks for potential collisions
   *
   * @param elevHeight The height of the elevator in meters
   * @param theta
   * @return position the wrist should be at in rads
   */
  private static OptionalDouble avoid(double elevHeight, double theta) {
    if (elevHeight < WristConstants.WRIST_AXEL_LOWER_LIMIT) {
      return OptionalDouble.empty();
    }
    double wristY = elevHeight - WristConstants.LENGTH * Math.sin(theta);
    if (wristY < SuperStructureConstants.COLLISION_HEIGHT) {
      return OptionalDouble.of(
          Math.asin(
              (elevHeight - SuperStructureConstants.COLLISION_HEIGHT) / WristConstants.LENGTH));
    }
    return OptionalDouble.of(theta);
  }

  public SuperStructure(SimCtx simCtx) {
    visualizer = new SuperStructureVisualizer();
    if (Robot.isReal()) {
      // wrist = new WristReal();
      // elevator = new ElevatorDisabled();
      elevator = new ElevatorReal();
      wrist = new WristDisabled();
    } else {
      wrist = new WristSim(simCtx);
      elevator = new ElevatorSim(simCtx);
    }
  }

  /**
   * Moves the superstructure to the desired position, needs to be called every cycle otherwise the
   * superstructure will not move
   *
   * @param elevatorMeters The height of the elevator in meters
   * @param wristRads The angle of the wrist in rads
   */
  public void goTo(double elevatorMeters, double wristRads) {
    elevatorMeters =
        MathUtil.clamp(elevatorMeters, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
    wristRads = MathUtil.clamp(wristRads, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
    OptionalDouble result = avoid(elevatorMeters, wristRads);
    if (result.isPresent()) {
      wristRads = result.getAsDouble();
    } else {
      wristRads = WristConstants.MAX_ANGLE;
      elevatorMeters = Math.max(elevatorMeters, WristConstants.WRIST_AXEL_LOWER_LIMIT);
    }
    elevator.gotoPosition(elevatorMeters);
    wrist.goToPosition(wristRads);
    visualizer.updateSetpoint(elevatorMeters, wristRads);
  }

  /**
   * Checks if the superstructure is at the desired position
   *
   * @param height The height of the elevator in meters
   * @param wristAngle The angle of the wrist in rads
   * @param elevatorTolerance The tolerance of the elevator in meters
   * @param wristTolerance The tolerance of the wrist in rads
   * @return True if the superstructure is at the desired position, false otherwise
   */
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
