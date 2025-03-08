package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberConstants;

public class ClimberCommands {
  public static Command stow(Climber climber) {
    return climber
        .startRun(() -> climber.setMagnetPower(false), () -> climber.setPivotPosition(0.0))
        .withName("ClimberStow");
  }

  public static Command stage(Climber climber) {
    return climber
        .startRun(
            () -> climber.setMagnetPower(true),
            () -> climber.setPivotPosition(ClimberConstants.PivotConstants.STAGE_ANGLE))
        .withName("ClimberStage");
  }

  public static Command climb(Climber climber) {
    return climber
        .startRun(
            () -> climber.setMagnetPower(true),
            () -> climber.setPivotPosition(ClimberConstants.PivotConstants.ASCEND_ANGLE))
        .withName("ClimberClimb");
  }

  public static Command testMagnet(Climber climber) {
    return climber
        .run(() -> climber.setMagnetPower(true))
        .finallyDo(() -> climber.setMagnetPower(false))
        .withName("ClimberTestMagnet");
  }
}
