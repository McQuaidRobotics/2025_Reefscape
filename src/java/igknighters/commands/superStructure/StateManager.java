package igknighters.commands.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.superStructure.Wrist.WristConstants;
import java.util.EnumMap;
import java.util.Set;

public class StateManager {

  private final SuperStructure superStructure;
  private SuperStructureState lastState = SuperStructureState.Stow;
  private final EnumMap<SuperStructureState, EnumMap<SuperStructureState, Transition>> transitions =
      new EnumMap<>(SuperStructureState.class);

  private static Command basicHoldAt(
      SuperStructure superStructure, double elevatorMeters, double wristRads) {
    return superStructure.run(() -> superStructure.goTo(elevatorMeters, wristRads));
  }

  private static Command basicMoveTo(
      SuperStructure superStructure,
      double elevatorMeters,
      double wristRads,
      double toleranceScalar) {
    return basicHoldAt(superStructure, elevatorMeters, wristRads)
        .until(
            () ->
                superStructure.isAt(
                    elevatorMeters,
                    wristRads,
                    ElevatorConstants.DEFAULT_TOLERANCE * toleranceScalar,
                    WristConstants.DEFAULT_TOLERANCE * toleranceScalar));
  }

  @FunctionalInterface
  public interface Transition {
    Command getCommand(
        SuperStructure superStructure, SuperStructureState from, SuperStructureState to);

    public static Transition DEFAULT =
        (superStructure, from, to) -> {
          return basicMoveTo(superStructure, to.elevatorMeters, to.wristRads, to.toleranceScalar);
        };
  }

  @SuppressWarnings("unused")
  private void addTransition(
      SuperStructureState from, SuperStructureState to, Transition transition) {
    transitions.putIfAbsent(from, new EnumMap<>(SuperStructureState.class));
    transitions.get(from).put(to, transition);
  }

  public StateManager(SuperStructure superStructure) {
    this.superStructure = superStructure;
  }

  private Command getTransitionCmd(
      SuperStructure superStructure, SuperStructureState from, SuperStructureState to) {
    if (transitions.containsKey(from) && transitions.get(from).containsKey(to)) {
      return transitions.get(from).get(to).getCommand(superStructure, from, to);
    } else {
      return Transition.DEFAULT.getCommand(superStructure, from, to);
    }
  }

  public Command moveTo(SuperStructureState to) {
    return Commands.defer(
            () -> getTransitionCmd(superStructure, this.lastState, to), Set.of(superStructure))
        .finallyDo(() -> this.lastState = to)
        .withName("MoveTo(" + to.name() + ")");
  }

  public Command holdAt(SuperStructureState to) {
    // return moveTo(to)
    //     .andThen(basicHoldAt(superStructure, to.elevatorMeters, to.wristRads))
    //     .withName("HoldAt(" + to.name() + ")");
    return basicHoldAt(superStructure, to.elevatorMeters, to.wristRads);
  }
}
