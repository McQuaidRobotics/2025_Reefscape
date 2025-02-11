package wayfinder.controllers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Per;
import wayfinder.controllers.Types.Constraints;
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

  private final double kP, kD;
  private final boolean replanning;

  private double prevError = 0;
  private State prevSetpoint = State.kZero;

  public TranslationController(double kP, double kD, boolean replanning) {
    this.kP = kP;
    this.kD = kD;
    this.replanning = replanning;
  }

  public TranslationController(
      Per<DistanceUnit, DistanceUnit> kP,
      Per<LinearVelocityUnit, LinearVelocityUnit> kD,
      boolean replanning) {
    this.kP = kP.in(Meters.per(Meters));
    this.kD = kD.in(MetersPerSecond.per(MetersPerSecond));
    this.replanning = replanning;
  }

  private double veloInDirection(FieldSpeeds speeds, Rotation2d targetDirection) {
    Rotation2d speedsDirection = speeds.direction();
    double dot =
        speedsDirection.getCos() * targetDirection.getCos()
            + speedsDirection.getSin() * targetDirection.getSin();
    return speeds.magnitude() * dot;
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
    final Rotation2d direction = measurement.minus(target).getAngle();
    final double velo = veloInDirection(measurementVelo, direction);

    State setpoint =
        DynamicTrapezoidProfile.calculate(
            period,
            replanning ? distance : prevSetpoint.position(),
            replanning ? velo : prevSetpoint.velocity(),
            0.0,
            0.0,
            constraints.maxVelocity(),
            constraints.maxAcceleration());

    double positionError = prevSetpoint.position() - distance;
    double errorOverTime = (positionError - prevError) / period;
    prevError = positionError;

    prevSetpoint = setpoint;

    double dirVelo = (kP * positionError) + (kD * errorOverTime) + setpoint.velocity();
    dirVelo = MathUtil.clamp(dirVelo, -constraints.maxVelocity(), constraints.maxVelocity());

    return new FieldSpeeds(dirVelo * direction.getCos(), dirVelo * direction.getSin(), 0.0);
  }

  public void reset(Translation2d measurement, FieldSpeeds measurementVelo, Translation2d target) {
    prevError = 0;
    final Rotation2d direction = measurement.minus(target).getAngle();
    final double distance = measurement.getDistance(target);
    prevSetpoint = new State(distance, veloInDirection(measurementVelo, direction));
  }
}
