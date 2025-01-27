package sham.shamController.unitSafeControl;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;

public sealed interface UnitFeedforward<O extends Unit, Q extends Unit> {
  public void logGains(EpilogueBackend logger);

  public interface AngularFF<O extends Unit> {
    public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate);

    public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate);

    public Measure<O> calculate(AngularVelocity goalRate);

    public default Measure<O> calculate(
        Angle current, AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return calculate(goalRate, goalRateRate);
    }

    public default Measure<O> calculate(
        Angle current, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return calculate(goalRate, nextGoalRate);
    }

    public default Measure<O> calculate(Angle current, AngularVelocity goalRate) {
      return calculate(goalRate);
    }
  }

  public interface LinearFF<O extends Unit> {
    public Measure<O> calculate(LinearVelocity goalRate, LinearAcceleration goalRateRate);

    public Measure<O> calculate(LinearVelocity goalRate, LinearVelocity nextGoalRate);

    public Measure<O> calculate(LinearVelocity goalRate);
  }

  public static final class SimpleFeedforward<O extends Unit>
      implements UnitFeedforward<O, AngleUnit>, AngularFF<O> {
    private final edu.wpi.first.math.controller.SimpleMotorFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, AngularVelocityUnit> kV;
    final Per<O, AngularAccelerationUnit> kA;

    public SimpleFeedforward(
        Measure<O> kS,
        Per<O, AngularVelocityUnit> kV,
        Per<O, AngularAccelerationUnit> kA,
        Time dt) {
      outputUnit = kS.baseUnit();
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      internalFeedforward =
          new edu.wpi.first.math.controller.SimpleMotorFeedforward(
              kS.baseUnitMagnitude(),
              kV.in(PerUnit.combine(outputUnit, RadiansPerSecond)),
              kA.in(PerUnit.combine(outputUnit, RadiansPerSecondPerSecond)),
              dt.in(Seconds));
    }

    public static SimpleFeedforward<VoltageUnit> forVoltage(
        AngleUnit inputUnit, double kS, double kV, double kA, Time dt) {
      return new SimpleFeedforward<VoltageUnit>(
          Volts.of(kS),
          Volts.per(inputUnit.per(Seconds)).ofNative(kV),
          Volts.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
          dt);
    }

    public static SimpleFeedforward<CurrentUnit> forCurrent(
        AngleUnit inputUnit, double kS, double kV, double kA, Time dt) {
      return new SimpleFeedforward<CurrentUnit>(
          Amps.of(kS),
          Amps.per(inputUnit.per(Seconds)).ofNative(kV),
          Amps.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
          dt);
    }

    @Override
    public void logGains(EpilogueBackend logger) {
      logger.log("kS", kS.baseUnitMagnitude());
      logger.log("kV", kV.baseUnitMagnitude());
      logger.log("kA", kA.baseUnitMagnitude());
      logger.log("outputUnit", outputUnit.name());
      logger.log("inputUnit", Radian.name());
      logger.log("feedforwardType", "Flywheel");
    }

    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  goalRate.in(RadiansPerSecond), goalRateRate.in(RadiansPerSecondPerSecond)));
    }

    @SuppressWarnings("unchecked")
    public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  goalRate.in(RadiansPerSecond), nextGoalRate.in(RadiansPerSecond)));
    }

    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(AngularVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(internalFeedforward.calculate(goalRate.in(RadiansPerSecond)));
    }
  }

  public static final class ElevatorFeedforward<O extends Unit>
      implements UnitFeedforward<O, DistanceUnit>, LinearFF<O> {
    private final edu.wpi.first.math.controller.ElevatorFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS, kG;
    final Per<O, LinearVelocityUnit> kV;
    final Per<O, LinearAccelerationUnit> kA;

    public ElevatorFeedforward(
        Measure<O> kS,
        Measure<O> kG,
        Per<O, LinearVelocityUnit> kV,
        Per<O, LinearAccelerationUnit> kA,
        Time dt) {
      outputUnit = kS.baseUnit();
      internalFeedforward =
          new edu.wpi.first.math.controller.ElevatorFeedforward(
              kS.baseUnitMagnitude(),
              kG.baseUnitMagnitude(),
              kV.in(PerUnit.combine(outputUnit, MetersPerSecond)),
              kA.in(PerUnit.combine(outputUnit, MetersPerSecondPerSecond)),
              dt.in(Seconds));
      this.kS = kS;
      this.kG = kG;
      this.kV = kV;
      this.kA = kA;
    }

    public static ElevatorFeedforward<VoltageUnit> forVoltage(
        DistanceUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
      return new ElevatorFeedforward<VoltageUnit>(
          Volts.of(kS),
          Volts.of(kG),
          Volts.per(inputUnit.per(Seconds)).ofNative(kV),
          Volts.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
          dt);
    }

    public static ElevatorFeedforward<CurrentUnit> forCurrent(
        DistanceUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
      return new ElevatorFeedforward<CurrentUnit>(
          Amps.of(kS),
          Amps.of(kG),
          Amps.per(inputUnit.per(Seconds)).ofNative(kV),
          Amps.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
          dt);
    }

    public static ElevatorFeedforwardAngularAdapter<VoltageUnit> forAngularVoltage(
        AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
      kV = inputUnit.of(kV).in(Radians);
      kA = inputUnit.of(kA).in(Radians);
      return new ElevatorFeedforward<VoltageUnit>(
              Volts.of(kS),
              Volts.of(kG),
              Volts.per(Meters.per(Seconds)).ofNative(kV),
              Volts.per(Meters.per(Seconds).per(Seconds)).ofNative(kA),
              dt)
          .withAngularInput(Meters.of(1.0));
    }

    public static ElevatorFeedforwardAngularAdapter<CurrentUnit> forAngularCurrent(
        AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
      kV = inputUnit.of(kV).in(Radians);
      kA = inputUnit.of(kA).in(Radians);
      return new ElevatorFeedforward<CurrentUnit>(
              Amps.of(kS),
              Amps.of(kG),
              Amps.per(Meters.per(Seconds)).ofNative(kV),
              Amps.per(Meters.per(Seconds).per(Seconds)).ofNative(kA),
              dt)
          .withAngularInput(Meters.of(1.0));
    }

    @Override
    public void logGains(EpilogueBackend logger) {
      logger.log("kS", kS);
      logger.log("kG", kG);
      logger.log("kV", kV);
      logger.log("kA", kA);
      logger.log("outputUnit", outputUnit.name());
      logger.log("inputUnit", Meters.name());
      logger.log("feedforwardType", "Elevator");
    }

    @Override
    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(LinearVelocity goalRate, LinearAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  goalRate.in(MetersPerSecond), goalRateRate.in(MetersPerSecondPerSecond)));
    }

    @Override
    @SuppressWarnings("unchecked")
    public Measure<O> calculate(LinearVelocity goalRate, LinearVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  goalRate.in(MetersPerSecond), nextGoalRate.in(MetersPerSecond)));
    }

    @Override
    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(LinearVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(internalFeedforward.calculate(goalRate.in(MetersPerSecond)));
    }

    public ElevatorFeedforwardAngularAdapter<O> withAngularInput(Distance spoolRadius) {
      return new ElevatorFeedforwardAngularAdapter<>(this, spoolRadius);
    }
  }

  public static final class ElevatorFeedforwardAngularAdapter<O extends Unit>
      implements UnitFeedforward<O, AngleUnit>, AngularFF<O> {
    private final ElevatorFeedforward<O> elevatorFeedforward;
    private final double spoolRadiusMeters;

    private ElevatorFeedforwardAngularAdapter(
        ElevatorFeedforward<O> elevatorFeedforward, Distance spoolRadius) {
      this.elevatorFeedforward = elevatorFeedforward;
      this.spoolRadiusMeters = spoolRadius.in(Meters);
    }

    @Override
    public void logGains(EpilogueBackend logger) {
      elevatorFeedforward.logGains(logger);
    }

    @Override
    public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return elevatorFeedforward.calculate(
          MetersPerSecond.of(goalRate.in(RadiansPerSecond) * spoolRadiusMeters),
          MetersPerSecondPerSecond.of(
              goalRateRate.in(RadiansPerSecondPerSecond) * spoolRadiusMeters));
    }

    @Override
    public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return elevatorFeedforward.calculate(
          MetersPerSecond.of(goalRate.in(RadiansPerSecond) * spoolRadiusMeters),
          MetersPerSecond.of(nextGoalRate.in(RadiansPerSecond) * spoolRadiusMeters));
    }

    @Override
    public Measure<O> calculate(AngularVelocity goalRate) {
      return elevatorFeedforward.calculate(
          MetersPerSecond.of(goalRate.in(RadiansPerSecond) * spoolRadiusMeters));
    }
  }

  public static final class ArmFeedforward<O extends Unit>
      implements UnitFeedforward<O, AngleUnit>, AngularFF<O> {
    private final edu.wpi.first.math.controller.ArmFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS, kG;
    final Per<O, AngularVelocityUnit> kV;
    final Per<O, AngularAccelerationUnit> kA;

    public ArmFeedforward(
        Measure<O> kS,
        Measure<O> kG,
        Per<O, AngularVelocityUnit> kV,
        Per<O, AngularAccelerationUnit> kA,
        Time dt) {
      outputUnit = kS.baseUnit();
      internalFeedforward =
          new edu.wpi.first.math.controller.ArmFeedforward(
              kS.baseUnitMagnitude(),
              kG.baseUnitMagnitude(),
              kV.in(PerUnit.combine(outputUnit, RadiansPerSecond)),
              kA.in(PerUnit.combine(outputUnit, RadiansPerSecondPerSecond)),
              dt.in(Seconds));
      this.kS = kS;
      this.kG = kG;
      this.kV = kV;
      this.kA = kA;
    }

    public static ArmFeedforward<VoltageUnit> forVoltage(
        AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
      return new ArmFeedforward<VoltageUnit>(
          Volts.of(kS),
          Volts.of(kG),
          Volts.per(inputUnit.per(Seconds)).ofNative(kV),
          Volts.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
          dt);
    }

    public static ArmFeedforward<CurrentUnit> forCurrent(
        AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
      return new ArmFeedforward<CurrentUnit>(
          Amps.of(kS),
          Amps.of(kG),
          Amps.per(inputUnit.per(Seconds)).ofNative(kV),
          Amps.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
          dt);
    }

    @Override
    public void logGains(EpilogueBackend logger) {
      logger.log("kS", kS);
      logger.log("kG", kG);
      logger.log("kV", kV);
      logger.log("kA", kA);
      logger.log("outputUnit", outputUnit.name());
      logger.log("inputUnit", Radian.name());
      logger.log("feedforwardType", "Arm");
    }

    @Override
    public Measure<O> calculate(AngularVelocity goalRate) {
      throw new UnsupportedOperationException();
    }

    @Override
    public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      throw new UnsupportedOperationException();
    }

    @Override
    public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      throw new UnsupportedOperationException();
    }

    @Override
    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(
        Angle currentAngle, AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  currentAngle.in(Radian),
                  goalRate.in(RadiansPerSecond),
                  goalRateRate.in(RadiansPerSecondPerSecond)));
    }

    @Override
    @SuppressWarnings("unchecked")
    public Measure<O> calculate(
        Angle currentAngle, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  currentAngle.in(Radian),
                  goalRate.in(RadiansPerSecond),
                  nextGoalRate.in(RadiansPerSecond)));
    }

    @Override
    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(Angle currentAngle, AngularVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  currentAngle.in(Radian), goalRate.in(RadiansPerSecond)));
    }
  }
}
