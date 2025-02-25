package igknighters.commands.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.EnumMap;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class StateManager {

  private final SuperStructure superStructure;
  private SuperStructureState lastState = SuperStructureState.Stow;
  private final EnumMap<SuperStructureState, EnumMap<SuperStructureState, Transition>> transitions =
      new EnumMap<>(SuperStructureState.class);
  private final BooleanSupplier holdingAlgae;

  public StateManager(SuperStructure superStructure, BooleanSupplier holdingAlgae) {
    this.superStructure = superStructure;
    this.holdingAlgae = holdingAlgae;
  }

  private Command basicHoldAt(
      SuperStructure superStructure, double elevatorMeters, double wristRads) {
    return superStructure.run(
        () -> superStructure.goTo(elevatorMeters, wristRads, holdingAlgae.getAsBoolean()));
  }

  @FunctionalInterface
  public interface Transition {
    Command getCommand(
        SuperStructure superStructure, SuperStructureState from, SuperStructureState to);
  }

  @SuppressWarnings("unused")
  private void addTransition(
      SuperStructureState from, SuperStructureState to, Transition transition) {
    transitions.putIfAbsent(from, new EnumMap<>(SuperStructureState.class));
    transitions.get(from).put(to, transition);
  }

  private Command getTransitionCmd(
      SuperStructure superStructure, SuperStructureState from, SuperStructureState to) {
    if (transitions.containsKey(from) && transitions.get(from).containsKey(to)) {
      return transitions.get(from).get(to).getCommand(superStructure, from, to);
    } else {
      return basicHoldAt(superStructure, to.elevatorMeters, to.wristRads);
    }
  }

  public Command moveTo(SuperStructureState to) {
    return Commands.defer(
            () -> getTransitionCmd(superStructure, this.lastState, to), Set.of(superStructure))
        .finallyDo(() -> this.lastState = to)
        .until(
            () ->
                superStructure.isAt(
                    to.elevatorMeters,
                    to.wristRads,
                    kElevator.DEFAULT_TOLERANCE * to.toleranceScalar,
                    kWrist.DEFAULT_TOLERANCE * to.toleranceScalar))
        .withName("MoveTo(" + to.name() + ")");
  }

  public Command holdAt(SuperStructureState to) {
    // return moveTo(to)
    //     .andThen(basicHoldAt(superStructure, to.elevatorMeters, to.wristRads))
    //     .withName("HoldAt(" + to.name() + ")");
    return basicHoldAt(superStructure, to.elevatorMeters, to.wristRads)
        .withName("HoldAt(" + to.name() + ")");
  }

  public Command home() {
    return superStructure
        .run(() -> superStructure.home(kWrist.MAX_ANGLE, 0.1))
        .until(superStructure::isHomed)
        .unless(superStructure::isHomed)
        .withName("HomeSuperStructure");
  }
}
