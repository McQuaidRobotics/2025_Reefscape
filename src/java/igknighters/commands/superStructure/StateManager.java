package igknighters.commands.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.EnumMap;
import java.util.Set;

public class StateManager {

  private SuperStructureState lastState = SuperStructureState.Stow;

  @FunctionalInterface
  public interface Transition {
    Command getCommand(
        SuperStructure superStructure, SuperStructureState from, SuperStructureState to);

    public static Transition DEFAULT =
        (superStructure, from, to) -> {
          return SuperStructureCommands.moveTo(
              superStructure, to.elevatorMeters, to.wristRads, to.toleranceScalar);
        };
  }

  private final EnumMap<SuperStructureState, EnumMap<SuperStructureState, Transition>> transitions =
      new EnumMap<>(SuperStructureState.class);

  private void addTransition(
      SuperStructureState from, SuperStructureState to, Transition transition) {
    transitions.putIfAbsent(from, new EnumMap<>(SuperStructureState.class));
    transitions.get(from).put(to, transition);
  }

  public void applyTransitions() {
    // add transitions here
  }

  private Command getTransitionCmd(
      SuperStructure superStructure, SuperStructureState from, SuperStructureState to) {
    if (transitions.containsKey(from) && transitions.get(from).containsKey(to)) {
      return transitions.get(from).get(to).getCommand(superStructure, from, to);
    } else {
      return Transition.DEFAULT.getCommand(superStructure, from, to);
    }
  }

  public Command moveTo(SuperStructure superStructure, SuperStructureState to) {
    return Commands.defer(
            () -> getTransitionCmd(superStructure, this.lastState, to), Set.of(superStructure))
        .finallyDo(() -> this.lastState = to)
        .withName("MoveTo(" + to.name() + ")");
  }
}
