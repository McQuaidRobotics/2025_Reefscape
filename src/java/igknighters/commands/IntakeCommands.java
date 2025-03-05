package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.ControlType;
import igknighters.subsystems.intake.Intake.Holding;

public class IntakeCommands {
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
    return runVoltage(intake, -3.0)
        .alongWith(Commands.run(() -> intake.setTryingToHold(Holding.CORAL)))
        .until(intake.isHolding(Holding.CORAL))
        .withName("IntakeCoral");
  }

  public static Command intakeAlgae(Intake intake) {
    return runVoltage(intake, -10.0)
        .alongWith(Commands.run(() -> intake.setTryingToHold(Holding.ALGAE)))
        .until(intake.isHolding(Holding.ALGAE))
        .withName("IntakeAlgae");
  }

  public static Command expel(Intake intake) {
    return runVoltage(intake, 3.0).withTimeout(0.3).withName("IntakeExpel");
  }

  public static Command bounce(Intake intake) {
    return Commands.sequence(
        IntakeCommands.runVoltage(intake, 1.0).withTimeout(0.1),
        IntakeCommands.runVoltage(intake, -12.0).withTimeout(0.15)
    );
  }

  public static Command holdAlgae(Intake intake) {
    return IntakeCommands.runCurrent(intake, -80.0)
      .withName("HoldAlgae");
  }

  public static Command holdCoral(Intake intake) {
    return IntakeCommands.runCurrent(intake, -25.0)
      .withName("HoldCoral");
  }
}
