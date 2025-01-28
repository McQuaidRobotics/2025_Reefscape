package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.State;
import wpilibExt.Speeds.FieldSpeeds;

public class TranslationController {
  private final double kP, kD;

  private double prevError = 0;
  private State prevSetpoint = State.kZero;

  public TranslationController(double kP, double kD) {
    this.kP = kP;
    this.kD = kD;
  }

  private double signSqr(double x) {
    return Math.copySign(Math.pow(x, 2.0), x);
  }

  public FieldSpeeds calculate(
      double period,
      Translation2d measurement,
      Translation2d target,
      double deadband,
      Constraints constraints) {

    // GlobalField.setObject(
    //     "Track", new Pose2d(measurement, Rotation2d.kZero), new Pose2d(target,
    // Rotation2d.kZero));

    final double distance = measurement.getDistance(target);
    if (distance < deadband + 0.001) {
      return FieldSpeeds.kZero;
    }
    final Rotation2d direction =
        new Rotation2d(measurement.getX() - target.getX(), measurement.getY() - target.getY());

    State setpoint =
        DynamicTrapezoidProfile.calculate(
            period,
            prevSetpoint,
            State.kZero,
            constraints.maxVelocity(),
            constraints.maxAcceleration());
    prevSetpoint = setpoint;

    double positionError = setpoint.position() - distance;
    prevError = positionError;
    double errorOverTime = (positionError - prevError) / period;
    double dirVelo =
        MathUtil.clamp(
            (kP * positionError) + (kD * errorOverTime) + setpoint.velocity(),
            -constraints.maxVelocity(),
            constraints.maxVelocity());

    return new FieldSpeeds(
        dirVelo * signSqr(direction.getCos()), dirVelo * signSqr(direction.getSin()), 0.0);
  }

  public void reset(Translation2d measurement, FieldSpeeds measurementVelo, Translation2d target) {
    prevError = 0;
    final Rotation2d direction = measurement.minus(target).getAngle();
    final double distance = measurement.getDistance(target);
    final double velo =
        measurementVelo.vx() * signSqr(direction.getCos())
            + measurementVelo.vy() * signSqr(direction.getSin());
    prevSetpoint = new State(distance, Math.abs(velo));
  }
}
