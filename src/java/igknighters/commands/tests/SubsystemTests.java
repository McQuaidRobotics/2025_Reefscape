package igknighters.commands.tests;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import igknighters.commands.ClimberCommands;
import igknighters.commands.SuperStructureCommands;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.superStructure.SuperStructureState;

public class SubsystemTests {
  private static final SuperStructureState[] STATE_TEST_ORDER =
      new SuperStructureState[] {
        SuperStructureState.AntiTilt,
        SuperStructureState.Processor,
        SuperStructureState.Net,
        SuperStructureState.IntakeHpFar,
        SuperStructureState.IntakeHpClose,
        SuperStructureState.ScoreL1,
        SuperStructureState.ScoreL2,
        SuperStructureState.ScoreL3,
        SuperStructureState.ScoreL4,
        SuperStructureState.AlgaeFloor,
        SuperStructureState.AlgaeL2,
        SuperStructureState.AlgaeL3,
        SuperStructureState.Stow
      };

  public static Command test(SuperStructure superStructure) {
    final SequentialCommandGroup testCommand =
        new SequentialCommandGroup(
            SuperStructureCommands.home(superStructure, true),
            SuperStructureCommands.moveTo(superStructure, SuperStructureState.Stow),
            SuperStructureCommands.home(superStructure, true));

    for (SuperStructureState state : STATE_TEST_ORDER) {
      testCommand.addCommands(
          SuperStructureCommands.moveTo(superStructure, state),
          SuperStructureCommands.holdAt(superStructure, state).withTimeout(1.5));
    }

    return testCommand;
  }

  public static Command test(Climber climber) {
    return Commands.sequence(
        ClimberCommands.stage(climber)
          .withTimeout(3.0),
        ClimberCommands.climb(climber)
          .until(() -> climber.isPivotAtPosition(-135.0 * Conv.DEGREES_TO_RADIANS, 0.1)),
        ClimberCommands.stow(climber)
          .withTimeout(3.0)
    );
  }
}
