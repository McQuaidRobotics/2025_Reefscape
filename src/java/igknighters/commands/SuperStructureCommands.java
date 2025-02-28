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
      SuperStructure superStructure, SuperStructureState state, double scalar) {
    return new Trigger(
        () ->
            superStructure.isAt(
                state.elevatorMeters,
                state.wristRads,
                kElevator.DEFAULT_TOLERANCE * state.toleranceScalar * scalar,
                kWrist.DEFAULT_TOLERANCE * state.toleranceScalar * scalar));
  }

  public static Command holdAt(SuperStructure superStructure, SuperStructureState state) {
    return superStructure
        .run(() -> superStructure.goTo(state.elevatorMeters, state.wristRads))
        .withName("HoldAt(" + state.name() + ")");
  }

  public static Command moveTo(SuperStructure superStructure, SuperStructureState state) {
    return holdAt(superStructure, state)
        .until(isAt(superStructure, state))
        .withName("MoveTo(" + state.name() + ")");
  }

  public static Command home(SuperStructure superStructure) {
    return superStructure
        .run(() -> superStructure.home(kWrist.MAX_ANGLE, 0.1))
        .until(superStructure::isHomed)
        .unless(superStructure::isHomed)
        .withName("HomeSuperStructure");
  }
}
