package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import igknighters.subsystems.superStructure.SuperStructureState;

public class SuperStructureCommands {
  public static Trigger isAt(SuperStructure superStructure, SuperStructureState state) {
    return new Trigger(
        () ->
            superStructure.isAt(
                state.elevatorMeters,
                state.wristRads,
                kElevator.DEFAULT_TOLERANCE * state.toleranceScalar,
                kWrist.DEFAULT_TOLERANCE * state.toleranceScalar));
  }

  public static Trigger isAt(
      SuperStructure superStructure, double elevatorMeters, double wristRads) {
    return new Trigger(
        () ->
            superStructure.isAt(
                elevatorMeters, wristRads, kElevator.DEFAULT_TOLERANCE, kWrist.DEFAULT_TOLERANCE));
  }

  public static Trigger isAt(
      SuperStructure superStructure, double elevatorMeters, double wristRads, double scalar) {
    return new Trigger(
        () ->
            superStructure.isAt(
                elevatorMeters,
                wristRads,
                kElevator.DEFAULT_TOLERANCE * scalar,
                kWrist.DEFAULT_TOLERANCE * scalar));
  }

  public static Trigger isElevatorAt(
      SuperStructure superStructure, double elevatorMeters, double tolerance) {
    return new Trigger(
        () -> superStructure.isAt(elevatorMeters, superStructure.wristAngle(), tolerance, 1000.0));
  }

  public static Trigger isWristAt(
      SuperStructure superStructure, double wristRads, double tolerance) {
    return new Trigger(
        () -> superStructure.isAt(superStructure.elevatorHeight(), wristRads, 1000.0, tolerance));
  }

  public enum MoveOrder {
    ELEVATOR_FIRST,
    WRIST_FIRST,
    SIMULTANEOUS
  }

  public static Command holdAt(
      SuperStructure superStructure, SuperStructureState state, MoveOrder order) {
    Command simultaneous =
        superStructure.run(() -> superStructure.goTo(state.elevatorMeters, state.wristRads));
    ;
    var tmp =
        switch (order) {
          case ELEVATOR_FIRST ->
              superStructure
                  .run(() -> superStructure.goTo(state.elevatorMeters, superStructure.wristAngle()))
                  .until(
                      () ->
                          superStructure.isAt(
                              state.elevatorMeters,
                              0.0,
                              kElevator.DEFAULT_TOLERANCE * state.toleranceScalar * 5.0,
                              1000.0))
                  .andThen(simultaneous);
          case WRIST_FIRST ->
              superStructure
                  .run(() -> superStructure.goTo(superStructure.elevatorHeight(), state.wristRads))
                  .until(
                      () ->
                          superStructure.isAt(
                              0.0,
                              state.wristRads,
                              1000.0,
                              kWrist.DEFAULT_TOLERANCE * state.toleranceScalar * 5.0))
                  .andThen(simultaneous);
          case SIMULTANEOUS -> simultaneous;
        };
    return tmp.withName("HoldAt(" + state.name() + ", " + order + ")");
  }

  public static Command holdAt(SuperStructure superStructure, SuperStructureState state) {
    return holdAt(superStructure, state, MoveOrder.SIMULTANEOUS);
  }

  public static Command moveTo(
      SuperStructure superStructure, SuperStructureState state, MoveOrder order) {
    return holdAt(superStructure, state, order)
        .until(isAt(superStructure, state))
        .withName("MoveTo(" + state.name() + ")");
  }

  public static Command moveTo(SuperStructure superStructure, SuperStructureState state) {
    return moveTo(superStructure, state, MoveOrder.SIMULTANEOUS);
  }

  public static Command home(SuperStructure superStructure, boolean force) {
    return superStructure
        .startRun(
            () -> superStructure.home(kWrist.MAX_ANGLE, 0.1, true),
            () -> superStructure.home(kWrist.MAX_ANGLE, 0.1, false))
        .until(superStructure::isHomed)
        .unless(() -> superStructure.isHomed() && !force)
        .withName("HomeSuperStructure");
  }
}
