package sham.shamController.unitSafeControl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;

public class UnitFeedback {
  /** A PD controller that uses units to ensure that the controller is used correctly. */
  public static class PIDFeedback<O extends Unit, Q extends Unit> {
    private final edu.wpi.first.math.controller.PIDController internalController;
    private final O outputUnit;
    private final Q inputUnit;

    public PIDFeedback(Per<O, Q> kP, Per<O, Q> kI, Per<O, VelocityUnit<Q>> kD) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController =
          new edu.wpi.first.math.controller.PIDController(
              kP.baseUnitMagnitude(), kI.baseUnitMagnitude(), kD.baseUnitMagnitude());
    }

    public static <O extends Unit> LinearPIDFeedback<O> forLinear(
        O outputUnit, DistanceUnit inputUnit, double kP, double kD) {
      return new LinearPIDFeedback<O>(
          PerUnit.combine(outputUnit, inputUnit).ofNative(kP),
          PerUnit.combine(outputUnit, inputUnit).ofNative(0.0),
          PerUnit.combine(outputUnit, inputUnit.per(Second)).ofNative(kD));
    }

    public static <O extends Unit> AngularPIDFeedback<O> forAngular(
        O outputUnit, AngleUnit inputUnit, double kP, double kD) {
      return new AngularPIDFeedback<O>(
          PerUnit.combine(outputUnit, inputUnit).ofNative(kP),
          PerUnit.combine(outputUnit, inputUnit).ofNative(0.0),
          PerUnit.combine(outputUnit, inputUnit.per(Second)).ofNative(kD));
    }

    public static <O extends Unit> LinearVelocityPIDFeedback<O> forLinearVelocity(
        O outputUnit, LinearVelocityUnit inputUnit, double kP) {
      return new LinearVelocityPIDFeedback<O>(
          PerUnit.combine(outputUnit, inputUnit).ofNative(kP),
          PerUnit.combine(outputUnit, inputUnit).ofNative(0.0),
          PerUnit.combine(outputUnit, inputUnit.per(Second)).ofNative(0.0));
    }

    public static <O extends Unit> AngularVelocityPIDFeedback<O> forAngularVelocity(
        O outputUnit, AngularVelocityUnit inputUnit, double kP) {
      return new AngularVelocityPIDFeedback<O>(
          PerUnit.combine(outputUnit, inputUnit).ofNative(kP),
          PerUnit.combine(outputUnit, inputUnit).ofNative(0.0),
          PerUnit.combine(outputUnit, inputUnit.per(Second)).ofNative(0.0));
    }

    public void logGains(EpilogueBackend logger) {
      logger.log("kP", internalController.getP());
      logger.log("kI", internalController.getI());
      logger.log("kD", internalController.getD());
      logger.log("outputUnit", outputUnit.name());
      logger.log("inputUnit", inputUnit.name());
    }

    @SuppressWarnings("unchecked")
    public Pair<Measure<O>, Measure<Q>> calculate(Measure<Q> measurement, Measure<Q> setpoint) {
      var o =
          (Measure<O>)
              outputUnit.of(
                  internalController.calculate(
                      measurement.baseUnitMagnitude(), setpoint.baseUnitMagnitude()));
      var error = (Measure<Q>) inputUnit.of(internalController.getError());
      return new Pair<>(o, error);
    }

    public PIDFeedback<O, Q> withTolerance(Measure<Q> tolerance) {
      internalController.setTolerance(tolerance.baseUnitMagnitude());
      return this;
    }

    public PIDFeedback<O, Q> withTolerance(
        Measure<Q> positionTolerance, Measure<Q> velocityTolerance) {
      internalController.setTolerance(
          positionTolerance.baseUnitMagnitude(), velocityTolerance.baseUnitMagnitude());
      return this;
    }

    public PIDFeedback<O, Q> withContinuousInput(Measure<Q> minimumInput, Measure<Q> maximumInput) {
      internalController.enableContinuousInput(
          minimumInput.baseUnitMagnitude(), maximumInput.baseUnitMagnitude());
      return this;
    }

    public O getOutputUnit() {
      return outputUnit;
    }

    public Q getInputUnit() {
      return inputUnit;
    }
  }

  public static class LinearPIDFeedback<O extends Unit> extends PIDFeedback<O, DistanceUnit> {

    public LinearPIDFeedback(
        Per<O, DistanceUnit> kP, Per<O, DistanceUnit> kI, Per<O, LinearVelocityUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Meters, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public Pair<Measure<O>, Measure<DistanceUnit>> calculate(
        Distance measurement, Distance setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  public static class LinearVelocityPIDFeedback<O extends Unit>
      extends PIDFeedback<O, LinearVelocityUnit> {
    public LinearVelocityPIDFeedback(
        Per<O, LinearVelocityUnit> kP,
        Per<O, LinearVelocityUnit> kI,
        Per<O, LinearAccelerationUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(MetersPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public Pair<Measure<O>, Measure<LinearVelocityUnit>> calculate(
        LinearVelocity measurement, LinearVelocity setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  public static class AngularPIDFeedback<O extends Unit> extends PIDFeedback<O, AngleUnit> {

    public AngularPIDFeedback(
        Per<O, AngleUnit> kP, Per<O, AngleUnit> kI, Per<O, AngularVelocityUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Radian, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public Pair<Measure<O>, Measure<AngleUnit>> calculate(Angle measurement, Angle setpoint) {
      return super.calculate(measurement, setpoint);
    }

    public AngularPIDFeedback<O> withContinuousAngularInput() {
      super.withContinuousInput(Radian.of(-Math.PI), Radian.of(Math.PI));
      return this;
    }
  }

  public static class AngularVelocityPIDFeedback<O extends Unit>
      extends PIDFeedback<O, AngularVelocityUnit> {
    public AngularVelocityPIDFeedback(
        Per<O, AngularVelocityUnit> kP,
        Per<O, AngularVelocityUnit> kI,
        Per<O, AngularAccelerationUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(RadiansPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public Pair<Measure<O>, Measure<AngularVelocityUnit>> calculate(
        AngularVelocity measurement, AngularVelocity setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }
}
