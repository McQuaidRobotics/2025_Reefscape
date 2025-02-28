package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.ControllerMode;
import wayfinder.controllers.Types.State;
import wpilibExt.Speeds.FieldSpeeds;

/**
 * A translation controller that implements feedback and trapezoidal motion profiling on the
 * distance to the target position and then extrapolates those into X and Y velocities.
 */
public class TranslationController {
  // These classes don't use the wpilib classes for a few reasons:
  // - in order for the trapezoidal profile in wpilib to be dynamic you have to recreate the object
  // each cycle
  // - the wpilib classes are harder to introspect for me with weird behavior
  // - i wanted to implement them myself to understand them better :3
  //
  // If you would like to use this in your own code feel free to implement this using the wpilib
  // classes

  private final double kP, kI, kD;
  private final ControllerMode mode;

  private double prevError, totalError;
  private State prevSetpoint = State.kZero;

  public TranslationController(double kP, double kI, double kD, ControllerMode mode) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.mode = mode;
  }

  public TranslationController(double kP, double kD, ControllerMode mode) {
    this(kP, 0, kD, mode);
  }

  private double veloInDirection(FieldSpeeds speeds, Rotation2d targetDirection) {
    if (speeds.magnitude() < 0.01) {
      return 0.0;
    }
    return speeds.magnitudeInDirection(targetDirection);
  }

  public FieldSpeeds calculate(
      double period,
      Translation2d measurement,
      FieldSpeeds measurementVelo,
      Translation2d target,
      double deadband,
      Constraints constraints) {

    final double distance = measurement.getDistance(target);
    if (distance < deadband + 0.001) {
      return FieldSpeeds.kZero;
    }

    final Rotation2d direction = target.minus(measurement).getAngle();
    final double velo = veloInDirection(measurementVelo, direction);

    if (mode == ControllerMode.UNPROFILED) {
      double positionError = distance;
      double errorDerivative = (positionError - prevError) / period;
      if (kI > 0) {
        totalError +=
            MathUtil.clamp(
                positionError * period,
                -constraints.maxAcceleration() * period / kI,
                constraints.maxAcceleration() * period / kI);
      }
      prevError = positionError;

      double dirVelo =
          MathUtil.clamp(
              (kP * positionError) + (kI * totalError) + (kD * errorDerivative),
              -constraints.maxVelocity(),
              constraints.maxVelocity());
      // double accel = (dirVelo - velo) / period;
      // if (Math.abs(accel) > constraints.maxAcceleration()) {
      //   dirVelo = velo + Math.copySign(constraints.maxAcceleration() * period, accel);
      // }

      return new FieldSpeeds(dirVelo * direction.getCos(), dirVelo * direction.getSin(), 0.0);
    } else {

      boolean replanning = mode == ControllerMode.REPLANNING;
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

      return new FieldSpeeds(dirVelo * direction.getCos(), dirVelo * direction.getSin(), 0.0);
    }
  }

  public void reset(Translation2d measurement, FieldSpeeds measurementVelo, Translation2d target) {
    prevError = 0;
    final Rotation2d direction = target.minus(measurement).getAngle();
    final double distance = measurement.getDistance(target);
    prevSetpoint = new State(-distance, veloInDirection(measurementVelo, direction));
  }
}
