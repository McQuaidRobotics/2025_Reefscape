package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.ControlType;
import igknighters.subsystems.intake.Intake.Holding;
import java.util.function.BooleanSupplier;

public class IntakeCommands {
  public static Trigger isHolding(Intake intake, Holding holding) {
    return new Trigger(() -> intake.getHolding() == holding);
  }

  public static Command runVoltage(Intake intake, double volts) {
    return intake
        .run(() -> intake.control(ControlType.VOLTAGE, volts))
        .withName("IntakeRunVoltage(" + volts + ")");
  }

  public static Command runCurrent(Intake intake, double current) {
    return intake
        .run(() -> intake.control(ControlType.CURRENT, current))
        .withName("IntakeRunCurrent(" + current + ")");
  }

  public static Command intakeCoral(Intake intake) {
    return runVoltage(intake, -7.5)
        .alongWith(Commands.run(() -> intake.setTryingToHold(Holding.CORAL)))
        .until(isHolding(intake, Holding.CORAL))
        .andThen(new ScheduleCommand(holdCoral(intake)))
        .withName("IntakeCoral");
  }

  public static Command intakeAlgae(Intake intake) {
    return runVoltage(intake, -12.0)
        .alongWith(Commands.run(() -> intake.setTryingToHold(Holding.ALGAE)))
        .withName("IntakeAlgae");
  }

  public static Command expel(Intake intake, BooleanSupplier isL1) {
    return Commands.either(
            runVoltage(intake, 14.0),
            Commands.either(runVoltage(intake, 7.0), runVoltage(intake, 12.0), isL1),
            isHolding(intake, Holding.ALGAE))
        .withName("Expel");
  }

  public static Command expel(Intake intake) {
    return expel(intake, () -> false);
  }

  public static Command bounce(Intake intake) {
    return Commands.sequence(
            IntakeCommands.runVoltage(intake, 3.0).withTimeout(0.10),
            intake.runOnce(() -> intake.setTryingToHold(Holding.CORAL)),
            IntakeCommands.runVoltage(intake, -12.0).withTimeout(0.2))
        .finallyDo(() -> intake.setTryingToHold(Holding.NONE))
        .withName("IntakeBounce");
  }

  public static Command holdAlgae(Intake intake) {
    return IntakeCommands.runCurrent(intake, -80.0)
        .until(isHolding(intake, Holding.ALGAE).negate())
        .withName("HoldAlgae");
  }

  public static Command holdCoral(Intake intake) {
    return IntakeCommands.runCurrent(intake, -12.5).withName("HoldCoral");
  }
}
