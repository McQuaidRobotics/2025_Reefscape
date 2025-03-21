package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.ControllerMode;
import wayfinder.controllers.Types.State;

public class RotationalController {
  // These classes don't use the wpilib classes for a few reasons:
  // - in order for the trapezoidal profile in wpilib to be dynamic you have to recreate the object
  // each cycle
  // - the wpilib classes are harder to introspect for me with weird behavior
  // - i wanted to implement them myself to understand them better :3
  //
  // If you would like to use this in your own code feel free to implement this using the wpilib
  // classes

  private final double kP, kD;
  private final ControllerMode mode;

  private double prevError = 0;
  private State prevSetpoint = State.kZero;

  public RotationalController(double kP, double kD, ControllerMode mode) {
    this.kP = kP;
    this.kD = kD;
    this.mode = mode;
  }

  public double calculate(
      double period,
      double measurement,
      double measurementVelo,
      double target,
      double deadband,
      Constraints constraints) {

    // this may not be needed but it doesn't hurt to have it
    measurement = MathUtil.angleModulus(measurement);
    target = MathUtil.angleModulus(target);

    // I uhh think this works most the time probably
    if (MathUtil.isNear(target, measurement, deadband)) {
      return 0.0;
    }

    if (mode == ControllerMode.UNPROFILED) {
      double positionError = MathUtil.angleModulus(target - measurement);
      double errorOverTime = (positionError - prevError) / period;
      prevError = positionError;

      double outVelo =
          MathUtil.clamp(
              (kP * positionError) + (kD * errorOverTime),
              -constraints.maxVelocity(),
              constraints.maxVelocity());
      double accel = (outVelo - measurementVelo) / period;
      if (Math.abs(accel) > constraints.maxAcceleration()) {
        outVelo = measurementVelo + Math.copySign(constraints.maxAcceleration() * period, accel);
      }

      return outVelo;
    } else {
      // ensure that the setpoint is always the shortest path to the target
      target = MathUtil.angleModulus(target - measurement) + measurement;
      double wrappedSetpoint =
          MathUtil.angleModulus(prevSetpoint.position() - measurement) + measurement;
      if (Math.abs(wrappedSetpoint - prevSetpoint.position()) > 0.001) {
        prevSetpoint = new State(wrappedSetpoint, prevSetpoint.velocity());
      }

      boolean replanning = mode == ControllerMode.REPLANNING;
      // calculate an intermediate setpoint based on constraints
      State setpoint =
          DynamicTrapezoidProfile.calculate(
              period,
              replanning ? measurement : prevSetpoint.position(),
              replanning ? measurementVelo : prevSetpoint.velocity(),
              target,
              0.0,
              constraints.maxVelocity(),
              constraints.maxAcceleration());

      // calculate the error and derivative of the error
      double positionError = MathUtil.angleModulus(prevSetpoint.position() - measurement);
      double errorOverTime = (positionError - prevError) / period;
      prevError = positionError;

      prevSetpoint = setpoint;

      // add feedback of the PD controller to the "feedforward" of the setpoint
      double ret = (kP * positionError) + (kD * errorOverTime) + setpoint.velocity();
      // ensure the feedback controller doesn't exceed the velocity constraints
      // (acceleration is harder to do and shouldn't really matter)
      return MathUtil.clamp(ret, -constraints.maxVelocity(), constraints.maxVelocity());
    }
  }

  public void reset(double measurement, double measurementVelo) {
    prevError = 0.0;
    prevSetpoint = new State(measurement, measurementVelo);
  }
}
