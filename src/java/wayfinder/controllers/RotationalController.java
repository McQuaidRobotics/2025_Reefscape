package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.State;

public class RotationalController {
  private final double kP, kD;
  private double prevError = 0;
  private State prevSetpoint = State.kZero;

  public RotationalController(double kP, double kD) {
    this.kP = kP;
    this.kD = kD;
  }

  public double calculate(
      double period, double measurement, double target, double deadband, Constraints constraints) {

    measurement = MathUtil.angleModulus(measurement);
    target = MathUtil.angleModulus(target);

    if (Math.abs(measurement - target) < deadband) {
      return 0.0;
    }

    target = MathUtil.angleModulus(target - measurement) + measurement;

    State setpoint =
        DynamicTrapezoidProfile.calculate(
            period,
            prevSetpoint.position(),
            prevSetpoint.velocity(),
            target,
            0.0,
            constraints.maxVelocity(),
            constraints.maxAcceleration());
    prevSetpoint = setpoint;

    double positionError = MathUtil.angleModulus(setpoint.position() - measurement);
    double errorOverTime = (positionError - prevError) / period;
    prevError = positionError;

    return (kP * positionError) + (kD * errorOverTime) + setpoint.velocity();
  }

  public void reset(double measurement, double measurementVelo, double target) {
    prevError = 0.0;
    prevSetpoint = new State(measurement, measurementVelo);
  }
}
