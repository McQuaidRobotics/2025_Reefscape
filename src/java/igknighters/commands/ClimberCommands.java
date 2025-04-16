package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberConstants;

public class ClimberCommands {
  public static Command stow(Climber climber) {
    return climber
        .run(() -> climber.setPivotPosition(ClimberConstants.kPivot.STOW_ANGLE))
        .withName("ClimberStow");
  }

  public static Command stage(Climber climber) {
    return climber
        .run(() -> climber.setPivotPosition(ClimberConstants.kPivot.STAGE_ANGLE))
        .withName("ClimberStage");
  }

  public static Command climb(Climber climber) {
    return climber
        .run(() -> climber.setPivotPosition(ClimberConstants.kPivot.ASCEND_ANGLE))
        .until(() -> climber.isPivotAtPosition(ClimberConstants.kPivot.ASCEND_ANGLE, 0.1))
        .andThen(climber.run(() -> climber.voltageOut(-0.475)))
        .withName("ClimberClimb");
  }
}
