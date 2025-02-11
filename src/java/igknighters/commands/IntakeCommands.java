package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    return runVoltage(intake, -3.0).until(intake.isHolding(Holding.CORAL));
  }

  public static Command intakeAlgae(Intake intake) {
    return runVoltage(intake, -12.0).until(intake.isHolding(Holding.CORAL));
  }
}
