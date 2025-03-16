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

  public static Command runTorque(Intake intake, double torque) {
    return intake
        .run(() -> intake.control(ControlType.TORQUE, torque))
        .withName("IntakeRunTorque(" + torque + ")");
  }

  public static Command runVelocity(Intake intake, double velocity) {
    return intake
        .run(() -> intake.control(ControlType.VELOCITY, velocity))
        .withName("IntakeRunVelocity(" + velocity + ")");
  }

  public static Command intakeCoral(Intake intake) {
    return runVoltage(intake, -3.8)
        .alongWith(Commands.run(() -> intake.setTryingToHold(Holding.CORAL)))
        .until(isHolding(intake, Holding.CORAL))
        .andThen(new ScheduleCommand(holdCoral(intake)))
        .withName("IntakeCoral");
  }

  public static Command intakeAlgae(Intake intake) {
    return runVoltage(intake, -10.0)
        .alongWith(Commands.run(() -> intake.setTryingToHold(Holding.ALGAE)))
        .withName("IntakeAlgae");
  }

  public static Command expel(Intake intake, BooleanSupplier isL1) {
    return Commands.either(
            runVoltage(intake, 5.0),
            Commands.either(runVoltage(intake, 2.0), runVoltage(intake, 3.5), isL1),
            isHolding(intake, Holding.ALGAE))
        .withName("Expel");
  }

  public static Command expel(Intake intake) {
    return expel(intake, () -> false);
  }

  public static Command bounce(Intake intake) {
    return Commands.sequence(
            IntakeCommands.runVoltage(intake, 1.0).withTimeout(0.1),
            intake.runOnce(() -> intake.setTryingToHold(Holding.CORAL)),
            IntakeCommands.runVoltage(intake, -12.0).withTimeout(0.15))
        .finallyDo(() -> intake.setTryingToHold(Holding.NONE))
        .withName("IntakeBounce");
  }

  public static Command holdAlgae(Intake intake) {
    return IntakeCommands.runCurrent(intake, -80.0)
        .until(isHolding(intake, Holding.ALGAE).negate())
        .withName("HoldAlgae");
  }

  public static Command holdCoral(Intake intake) {
    return IntakeCommands.runCurrent(intake, -35.0)
        .until(isHolding(intake, Holding.CORAL).negate())
        .withName("HoldCoral");
  }
}
