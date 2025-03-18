package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wayfinder.controllers.Types.State;

public abstract class RotationalController
    implements Controller<Rotation2d, Double, Rotation2d, Constraints> {
  // These classes don't use the wpilib classes for a few reasons:
  // - in order for the trapezoidal profile in wpilib to be dynamic you have to recreate the object
  // each cycle
  // - the wpilib classes are harder to introspect for me with weird behavior
  // - i wanted to implement them myself to understand them better :3
  //
  // If you would like to use this in your own code feel free to implement this using the wpilib
  // classes

  private static boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
    if (Math.abs(toleranceRadians) > Math.PI) {
      return true;
    }
    double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
    // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi], as pre-checked
    // above.
    return dot > Math.cos(toleranceRadians);
  }

  protected final double kP, kD;

  private RotationalController(double kP, double kD) {
    this.kP = kP;
    this.kD = kD;
  }

  @Override
  public abstract Double calculate(
      double period,
      Rotation2d measurement,
      Double measurementVelo,
      Rotation2d target,
      Constraints constraints);

  @Override
  public abstract void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target);

  @Override
  public abstract boolean isDone(Rotation2d measurement, Rotation2d target);

  private static class Profiled extends RotationalController {
    private final boolean replanning;

    private double prevError = 0;
    private State prevSetpoint = State.kZero;

    public Profiled(double kP, double kD, boolean replanning) {
      super(kP, kD);
      this.replanning = replanning;
    }

    public boolean isDone(Rotation2d measurement, Rotation2d target) {
      return MathUtil.isNear(prevSetpoint.position(), target.getRadians(), 0.001)
          && MathUtil.isNear(prevSetpoint.velocity(), 0.0, 0.01);
    }

    public Double calculate(
        double period,
        Rotation2d measurementGeom,
        Double measurementVelo,
        Rotation2d targetGeom,
        Constraints constraints) {
      if (isDone(measurementGeom, targetGeom)) {
        return 0.0;
      }

      double measurement = measurementGeom.getRadians();
      double target = targetGeom.getRadians();

      // this may not be needed but it doesn't hurt to have it
      measurement = MathUtil.angleModulus(measurement);
      target = MathUtil.angleModulus(target);

      // ensure that the setpoint is always the shortest path to the target
      target = MathUtil.angleModulus(target - measurement) + measurement;
      double wrappedSetpoint =
          MathUtil.angleModulus(prevSetpoint.position() - measurement) + measurement;
      if (Math.abs(wrappedSetpoint - prevSetpoint.position()) > 0.001) {
        prevSetpoint = new State(wrappedSetpoint, prevSetpoint.velocity());
      }

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

    public void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target) {
      prevError = 0.0;
      prevSetpoint = new State(measurement.getRadians(), measurementVelo);
    }
  }

  private static class UnProfiled extends RotationalController {
    private final double deadband;

    private double prevError;

    public UnProfiled(double kP, double kD, double deadband) {
      super(kP, kD);
      this.deadband = deadband;
    }

    public boolean isDone(Rotation2d measurement, Rotation2d target) {
      return withinTolerance(measurement, target, deadband);
    }

    public void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target) {
      prevError = 0;
    }

    public Double calculate(
        double period,
        Rotation2d measurementGeom,
        Double measurementVelo,
        Rotation2d targetGeom,
        Constraints constraints) {

      if (isDone(measurementGeom, targetGeom)) {
        return 0.0;
      }
      double measurement = measurementGeom.getRadians();
      double target = targetGeom.getRadians();

      double positionError = MathUtil.angleModulus(target - measurement);
      double errorOverTime = (positionError - prevError) / period;
      prevError = positionError;

      return (kP * positionError) + (kD * errorOverTime);
    }
  }

  public static RotationalController profiled(double kP, double kD, boolean replanning) {
    return new Profiled(kP, kD, replanning);
  }

  public static RotationalController unprofiled(double kP, double kD, double deadband) {
    return new UnProfiled(kP, kD, deadband);
  }
}
