package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import monologue.Monologue;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
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

    private double prevError, totalError;
    private State prevSetpoint = State.kZero;

    public Profiled(double kP, double kI, double kD, boolean replanning) {
      super(kP, kI, kD);
      this.replanning = replanning;
    }

    @Override
    public boolean isDone(Translation2d measurement, Translation2d target) {
      return MathUtil.isNear(prevSetpoint.position(), 0.0, 0.001)
          && MathUtil.isNear(prevSetpoint.velocity(), 0.0, 0.01);
    }

    @Override
    public void reset(Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
      prevError = 0;
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

      Monologue.log("TranslationController", "Profiled");

      if (isDone(measurement, target)) {
        return Velocity2d.kZero;
      }

      final double distance = measurement.getDistance(target);
      final Rotation2d direction = target.minus(measurement).getAngle();
      final double velo = measurementVelo.speedInDirection(direction);

      State setpoint =
          DynamicTrapezoidProfile.calculate(
              period,
              replanning ? -distance : prevSetpoint.position(),
              replanning ? velo : prevSetpoint.velocity(),
              0.0,
              0.0,
              constraints.maxVelocity(),
              constraints.maxAcceleration());

      double positionError = distance + prevSetpoint.position();

      Monologue.log("ProfiledError", positionError);

      double errorDerivative = (positionError - prevError) / period;
      if (kI > 0) {
        totalError +=
            MathUtil.clamp(
                positionError * period,
                -constraints.maxAcceleration() * period / kI,
                constraints.maxAcceleration() * period / kI);
      }
      prevError = positionError;
      prevSetpoint = setpoint;

      double dirVelo =
          (kP * positionError) + (kI * totalError) + (kD * errorDerivative) + setpoint.velocity();
      dirVelo = MathUtil.clamp(dirVelo, -constraints.maxVelocity(), constraints.maxVelocity());

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

      Monologue.log("TranslationController", "UnProfiled");

      if (isDone(measurement, target)) {
        return Velocity2d.kZero;
      }

      final double distance = measurement.getDistance(target);
      final Rotation2d direction = target.minus(measurement).getAngle();

      double positionError = distance;
      Monologue.log("UnProfiledError", positionError);
      double errorDerivative = (positionError - prevError) / period;
      if (kI > 0) {
        totalError += positionError * period;
      }
      prevError = positionError;

      double dirVelo = (kP * positionError) + (kI * totalError) + (kD * errorDerivative);

      return new Velocity2d(dirVelo * direction.getCos(), dirVelo * direction.getSin());
    }
  }

  public static TranslationController profiled(
      double kP, double kI, double kD, boolean replanning) {
    return new Profiled(kP, kI, kD, replanning);
  }

  public static TranslationController unprofiled(double kP, double kI, double kD, double deadband) {
    return new UnProfiled(kP, kI, kD, deadband);
  }
}
