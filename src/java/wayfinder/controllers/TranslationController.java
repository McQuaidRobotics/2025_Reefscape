package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wayfinder.controllers.Framework.Controller;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.State;
import wpilibExt.Velocity2d;

/**
 * A translation controller that implements feedback and trapezoidal motion profiling on the
 * distance to the target position and then extrapolates those into X and Y velocities.
 */
public abstract class TranslationController
    implements Controller<Translation2d, Velocity2d, Translation2d, Constraints> {
  // These classes don't use the wpilib classes for a few reasons:
  // - in order for the trapezoidal profile in wpilib to be dynamic you have to recreate the object
  // each cycle
  // - the wpilib classes are harder to introspect for me with weird behavior
  // - i wanted to implement them myself to understand them better :3
  //
  // If you would like to use this in your own code feel free to implement this using the wpilib
  // classes

  @Override
  public abstract Velocity2d calculate(
      double period,
      Translation2d measurement,
      Velocity2d measurementVelo,
      Translation2d target,
      Constraints constraints);

  @Override
  public abstract void reset(
      Translation2d measurement, Velocity2d measurementVelo, Translation2d target);

  @Override
  public abstract boolean isDone(Translation2d measurement, Translation2d target);

  protected final double kP, kI, kD;

  private TranslationController(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  private static final class Profiled extends TranslationController {
    private final boolean replanning;
    private final double deadband;

    private double prevError, totalError;
    private State prevSetpoint = State.kZero;

    Profiled(double kP, double kI, double kD, boolean replanning, double deadband) {
      super(kP, kI, kD);
      this.replanning = replanning;
      this.deadband = deadband;
    }

    @Override
    public boolean isDone(Translation2d measurement, Translation2d target) {
      if (replanning) {
        return measurement.getDistance(target) < deadband;
      } else {
        return prevSetpoint.isNear(State.kZero, 0.001, 0.01);
      }
    }

    @Override
    public void reset(Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
      prevError = 0;
      totalError = 0;
      final Rotation2d direction = target.minus(measurement).getAngle();
      final double distance = measurement.getDistance(target);
      prevSetpoint = new State(-distance, measurementVelo.speedInDirection(direction));
    }

    @Override
    public Velocity2d calculate(
        double period,
        Translation2d measurement,
        Velocity2d measurementVelo,
        Translation2d target,
        Constraints constraints) {

      if (isDone(measurement, target)) {
        return Velocity2d.kZero;
      }

      final double distance = measurement.getDistance(target);
      final Rotation2d direction = target.minus(measurement).getAngle();

      State setpoint =
          DynamicTrapezoidProfile.calculate(
              period,
              prevSetpoint.position(),
              prevSetpoint.velocity(),
              0.0,
              0.0,
              constraints.maxVelocity(),
              constraints.maxAcceleration());

      double positionError = distance + prevSetpoint.position();

      double errorDerivative = (positionError - prevError) / period;
      if (kI > 0) {
        totalError +=
            MathUtil.clamp(
                positionError * period,
                -constraints.maxAcceleration() * period / kI,
                constraints.maxAcceleration() * period / kI);
      }
      if (replanning && prevSetpoint.isNear(setpoint, 0.01, 0.01)) {
        prevSetpoint = new State(-distance, measurementVelo.speedInDirection(direction));
        prevError = 0;
        totalError = 0;
      } else {
        prevError = positionError;
        prevSetpoint = setpoint;
      }

      double dirVelo =
          (kP * positionError) + (kI * totalError) + (kD * errorDerivative) + setpoint.velocity();

      return new Velocity2d(dirVelo * direction.getCos(), dirVelo * direction.getSin());
    }
  }

  private static final class UnProfiled extends TranslationController {
    private final double deadband;

    private double prevError, totalError;

    public UnProfiled(double kP, double kI, double kD, double deadband) {
      super(kP, kI, kD);
      this.deadband = deadband;
    }

    @Override
    public boolean isDone(Translation2d measurement, Translation2d target) {
      return measurement.getDistance(target) < deadband;
    }

    public void reset(Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
      prevError = 0;
    }

    @Override
    public Velocity2d calculate(
        double period,
        Translation2d measurement,
        Velocity2d measurementVelo,
        Translation2d target,
        Constraints constraints) {

      if (isDone(measurement, target)) {
        return Velocity2d.kZero;
      }

      final double distance = measurement.getDistance(target);
      final Rotation2d direction = target.minus(measurement).getAngle();

      double positionError = distance;
      double errorDerivative = (positionError - prevError) / period;
      if (kI > 0) {
        totalError += positionError * period;
      }
      prevError = positionError;

      double dirVelo = (kP * positionError) + (kI * totalError) + (kD * errorDerivative);

      return new Velocity2d(dirVelo * direction.getCos(), dirVelo * direction.getSin());
    }
  }

  /**
   * Returns a profiled translation controller with the given gains.
   *
   * <p>This controller will finish once the profile is done, not when the target is reached. That
   * means the PID gains for this profiled controller are based upon the moving setpoint, not the
   * target position. Measuring error of a practically achievable setpoint allows the system to stay
   * stable with greater gains.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @return a profiled translation controller
   */
  public static TranslationController profiled(double kP, double kI, double kD) {
    return new Profiled(kP, kI, kD, false, 0.0);
  }

  /**
   * Returns a replanning profiled translation controller with the given gains.
   *
   * <p>A replanning profiled controller is a mix of a {@link #profiled(double, double, double)}
   * controller and {@link #unprofiled(double, double, double, double)} controller. The replanning
   * profiled controller will finish once the target is reached, not when the profile is done. If
   * the profile finishes before the target is reached, the controller will plan a new profile to
   * the target. This allows the controller to be tolerant of latent systems and error while still
   * largely respecting the constraints of the profile.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @param deadband the deadband for determining if the target is reached
   * @return a replanning profiled translation controller
   */
  public static TranslationController replanningProfiled(
      double kP, double kI, double kD, double deadband) {
    return new Profiled(kP, kI, kD, true, deadband);
  }

  /**
   * Returns a translation controller that does not use motion profiling. This controller will
   * finish once the target is reached.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @param deadband the deadband for determining if the target is reached
   * @return a translation controller that does not use motion profiling
   */
  public static TranslationController unprofiled(double kP, double kI, double kD, double deadband) {
    return new UnProfiled(kP, kI, kD, deadband);
  }

  /**
   * Wraps a generic controller in a TranslationController to comply with the type system.
   *
   * @param controller the controller to wrap
   * @return a wrapped controller
   */
  public static TranslationController wrap(
      Controller<Translation2d, Velocity2d, Translation2d, Constraints> controller) {
    return new TranslationController(0.0, 0.0, 0.0) {
      @Override
      public Velocity2d calculate(
          double period,
          Translation2d measurement,
          Velocity2d measurementVelo,
          Translation2d target,
          Constraints constraints) {
        return controller.calculate(period, measurement, measurementVelo, target, constraints);
      }

      @Override
      public void reset(
          Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
        controller.reset(measurement, measurementVelo, target);
      }

      @Override
      public boolean isDone(Translation2d measurement, Translation2d target) {
        return controller.isDone(measurement, target);
      }
    };
  }
}
