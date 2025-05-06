package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import wayfinder.controllers.Framework.Controller;
import wayfinder.controllers.Types.Constraints;
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
    private final double deadband;

    private double prevError = 0;
    private State prevSetpoint = State.kZero;

    Profiled(double kP, double kD, boolean replanning, double deadband) {
      super(kP, kD);
      this.replanning = replanning;
      this.deadband = deadband;
    }

    public boolean isDone(Rotation2d measurement, Rotation2d target) {
      if (replanning) {
        return withinTolerance(measurement, target, deadband);
      } else {
        return MathUtil.isNear(prevSetpoint.position(), target.getRadians(), 0.001)
            && MathUtil.isNear(prevSetpoint.velocity(), 0.0, 0.01);
      }
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
              prevSetpoint.position(),
              prevSetpoint.velocity(),
              target,
              0.0,
              constraints.maxVelocity(),
              constraints.maxAcceleration());

      // calculate the error and derivative of the error
      double positionError = MathUtil.angleModulus(prevSetpoint.position() - measurement);
      double errorOverTime = (positionError - prevError) / period;
      if (replanning && prevSetpoint.isNear(setpoint, 0.01, 0.01)) {
        reset(measurementGeom, measurementVelo, targetGeom);
      } else {
        prevError = positionError;
        prevSetpoint = setpoint;
      }

      // add feedback of the PD controller to the "feedforward" of the setpoint
      return (kP * positionError) + (kD * errorOverTime) + setpoint.velocity();
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

  /**
   * Returns a profiled rotation controller with the given gains.
   *
   * <p>This controller will finish once the profile is done, not when the target is reached. That
   * means the PID gains for this profiled controller are based upon the moving setpoint, not the
   * target position. Measuring error of a practically achievable setpoint allows the system to stay
   * stable with greater gains.
   *
   * @param kP the proportional gain
   * @param kD the derivative gain
   * @return a profiled rotation controller
   */
  public static RotationalController profiled(double kP, double kD) {
    return new Profiled(kP, kD, false, 0.0);
  }

  /**
   * Returns a replanning profiled rotation controller with the given gains.
   *
   * <p>A replanning profiled controller is a mix of a {@link #profiled(double, double)} controller
   * and {@link #unprofiled(double, double, double)} controller. The replanning profiled controller
   * will finish once the target is reached, not when the profile is done. If the profile finishes
   * before the target is reached, the controller will plan a new profile to the target. This allows
   * the controller to be tolerant of latent systems and error while still largely respecting the
   * constraints of the profile.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @param deadband the deadband for determining if the target is reached
   * @return a replanning profiled rotation controller
   */
  public static RotationalController replanningProfiled(double kP, double kD, double deadband) {
    return new Profiled(kP, kD, true, deadband);
  }

  /**
   * Returns a rotation controller that does not use motion profiling. This controller will finish
   * once the target is reached.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @param deadband the deadband for determining if the target is reached
   * @return a rotation controller that does not use motion profiling
   */
  public static RotationalController unprofiled(double kP, double kD, double deadband) {
    return new UnProfiled(kP, kD, deadband);
  }

  /**
   * Wraps a generic controller in a RotationalController to comply with the type system.
   *
   * @param controller the controller to wrap
   * @return a wrapped controller
   */
  public static RotationalController wrap(
      Controller<Rotation2d, Double, Rotation2d, Constraints> controller) {
    return new RotationalController(0.0, 0.0) {
      @Override
      public Double calculate(
          double period,
          Rotation2d measurement,
          Double measurementVelo,
          Rotation2d target,
          Constraints constraints) {
        return controller.calculate(period, measurement, measurementVelo, target, constraints);
      }

      @Override
      public void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target) {
        controller.reset(measurement, measurementVelo, target);
      }

      @Override
      public boolean isDone(Rotation2d measurement, Rotation2d target) {
        return controller.isDone(measurement, target);
      }
    };
  }
}
