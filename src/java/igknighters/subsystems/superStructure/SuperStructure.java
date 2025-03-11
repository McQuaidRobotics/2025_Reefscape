package igknighters.subsystems.superStructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.subsystems.SharedState;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorReal;
import igknighters.subsystems.superStructure.Elevator.ElevatorSim;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import igknighters.subsystems.superStructure.Wrist.Wrist;
import igknighters.subsystems.superStructure.Wrist.WristReal;
import igknighters.subsystems.superStructure.Wrist.WristSim;
import java.util.Optional;
import monologue.Annotations.Log;

public class SuperStructure implements ExclusiveSubsystem {
  private static final Optional<Constraints> ALGAE_WRIST_CONSTRAINTS =
      Optional.of(new Constraints(kWrist.ALGAE_MAX_VELOCITY, kWrist.ALGAE_MAX_ACCELERATION));

  private final SharedState shared;
  private final SuperStructureVisualizer visualizer;

  @Log(key = "Wrist")
  private final Wrist wrist;

  @Log(key = "Elevator")
  private final Elevator elevator;

  // /**
  //  * Checks for potential collisions
  //  *
  //  * @param elevHeight The height of the elevator in meters
  //  * @param theta
  //  * @return position the wrist should be at in rads
  //  */
  // private static double avoid(double elevHeight, double theta) {
  //   double wristY = elevHeight - kWrist.LENGTH * Math.sin(theta);
  //   if (wristY < SuperStructureConstants.COLLISION_HEIGHT) {
  //     return Math.asin((elevHeight - SuperStructureConstants.COLLISION_HEIGHT) / kWrist.LENGTH);
  //   }
  //   return theta;
  // }

  public SuperStructure(SharedState shared, SimCtx simCtx) {
    this.shared = shared;
    visualizer = new SuperStructureVisualizer();
    if (Robot.isReal()) {
      wrist = new WristReal();
      // elevator = new ElevatorDisabled();
      elevator = new ElevatorReal();
      // wrist = new WristDisabled();
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
    wrist.log("initialTargetRads", wristRads);
    elevator.log("initialTargetMeters", elevatorMeters);
    if (!isHomed()) {
      return;
    }
    elevatorMeters = MathUtil.clamp(elevatorMeters, kElevator.MIN_HEIGHT, kElevator.MAX_HEIGHT);
    if (shared.holdingAlgae) {
      wristRads = MathUtil.clamp(wristRads, kWrist.ALGAE_MAX_ANGLE, kWrist.MIN_ANGLE);
      wrist.goToPosition(wristRads, ALGAE_WRIST_CONSTRAINTS);
    } else {
      wristRads = MathUtil.clamp(wristRads, kWrist.MAX_ANGLE, kWrist.MIN_ANGLE);
      wrist.goToPosition(wristRads);
    }
    elevator.gotoPosition(elevatorMeters);
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

  public void home(double wristAngle, double wristTolerance, boolean force) {
    if (force) {
      elevator.resetHomed();
    }
    wrist.goToPosition(wristAngle);
    if (wrist.isAtPosition(wristAngle, wristTolerance)) {
      elevator.home();
    }
    visualizer.updateSetpoint(kElevator.MIN_HEIGHT, wristAngle);
  }

  public boolean isHomed() {
    return elevator.isHomed();
  }

  public double elevatorHeight() {
    return elevator.position();
  }

  public double wristAngle() {
    return wrist.position();
  }

  @Override
  public void periodic() {
    elevator.periodic();
    wrist.periodic();
    visualizer.updatePosition(elevator.position(), wrist.position());
    shared.elevatorHeight = elevator.position();
  }
}
