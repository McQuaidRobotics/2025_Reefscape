package igknighters.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.Intake.Holding;

public class IntakeCommands {
  public static Command runVoltage(Intake intake, double volts) {
    return intake.run(() -> intake.runAtVoltage(volts));
  }

  public static Command runTorque(Intake intake, double torque) {
    return intake.run(() -> intake.runAtTorque(torque));
  }

  public static Command intakeCoral(Intake intake) {
    return runVoltage(intake, -3.0)
        .until(intake.isHolding(Holding.CORAL));
  }

  public static Command intakeAlgae(Intake intake) {
    return runVoltage(intake, -12.0)
        .until(intake.isHolding(Holding.CORAL));
  }
}
