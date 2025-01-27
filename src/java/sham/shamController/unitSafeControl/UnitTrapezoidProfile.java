package sham.shamController.unitSafeControl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.ProceduralStructGenerator;
import wpilibExt.MeasureMath;

public class UnitTrapezoidProfile<DIM extends Unit> {
  private final Measure<DIM> maxValue;
  private final Velocity<DIM> maxSlew;
  private final Acceleration<DIM> maxSlewSlew;

  public record State<DIM extends Unit>(Measure<DIM> value, Velocity<DIM> slew)
      implements StructSerializable {
    public static State<AngleUnit> of(Angle position, AngularVelocity velocity) {
      return new State<>(
          position, VelocityUnit.combine(Radian, Second).of(velocity.in(RadiansPerSecond)));
    }

    public static State<DistanceUnit> of(Distance position, LinearVelocity velocity) {
      return new State<>(
          position, VelocityUnit.combine(Meters, Second).of(velocity.in(MetersPerSecond)));
    }

    public static State<AngularVelocityUnit> of(
        AngularVelocity velocity, AngularAcceleration acceleration) {
      return new State<>(
          velocity,
          VelocityUnit.combine(RadiansPerSecond, Second)
              .of(acceleration.in(RadiansPerSecondPerSecond)));
    }

    public static State<LinearVelocityUnit> of(
        LinearVelocity velocity, LinearAcceleration acceleration) {
      return new State<>(
          velocity,
          VelocityUnit.combine(MetersPerSecond, Second)
              .of(acceleration.in(MetersPerSecondPerSecond)));
    }

    @SuppressWarnings("rawtypes")
    public static final Struct<State> struct = ProceduralStructGenerator.genRecord(State.class);
  }

  private UnitTrapezoidProfile(
      Measure<DIM> maxValue, Velocity<DIM> maxSlew, Acceleration<DIM> maxSlewSlew) {
    this.maxValue = maxValue;
    this.maxSlew = maxSlew;
    this.maxSlewSlew = maxSlewSlew;
  }

  public static UnitTrapezoidProfile<AngleUnit> forAngle(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    final VelocityUnit<AngleUnit> vu = VelocityUnit.combine(Radian, Second);
    return new UnitTrapezoidProfile<>(
        Radian.of(10000000000.0),
        vu.of(maxVelocity.in(RadiansPerSecond)),
        AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(RadiansPerSecondPerSecond)));
  }

  public static UnitTrapezoidProfile<DistanceUnit> forDistance(
      LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
    final VelocityUnit<DistanceUnit> vu = VelocityUnit.combine(Meters, Second);
    return new UnitTrapezoidProfile<>(
        Meters.of(10000000000.0),
        vu.of(maxVelocity.in(MetersPerSecond)),
        AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(MetersPerSecondPerSecond)));
  }

  public static UnitTrapezoidProfile<AngularVelocityUnit> forAngularVelocity(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    final VelocityUnit<AngularVelocityUnit> vu = VelocityUnit.combine(RadiansPerSecond, Second);
    return new UnitTrapezoidProfile<>(
        maxVelocity,
        vu.of(maxAcceleration.in(RadiansPerSecondPerSecond)),
        AccelerationUnit.combine(vu, Second).of(10000000.0));
  }

  public static UnitTrapezoidProfile<LinearVelocityUnit> forLinearVelocity(
      LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
    final VelocityUnit<LinearVelocityUnit> vu = VelocityUnit.combine(MetersPerSecond, Second);
    return new UnitTrapezoidProfile<>(
        maxVelocity,
        vu.of(maxAcceleration.in(MetersPerSecondPerSecond)),
        AccelerationUnit.combine(vu, Second).of(10000000.0));
  }

  public void logConstraints(EpilogueBackend logger) {
    logger.log("maxValue", maxValue.baseUnitMagnitude());
    logger.log("maxSlew", maxSlew.baseUnitMagnitude());
    logger.log("maxSlewSlew", maxSlewSlew.baseUnitMagnitude());
    logger.log("constraintUnit", maxValue.unit().name());
  }

  @SuppressWarnings("unchecked")
  public State<DIM> calculate(State<DIM> current, State<DIM> goal, Time deltaTime) {
    final edu.wpi.first.math.trajectory.TrapezoidProfile internalProfile =
        new edu.wpi.first.math.trajectory.TrapezoidProfile(
            new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                maxSlew.baseUnitMagnitude(), maxSlewSlew.baseUnitMagnitude()));
    var internalState =
        internalProfile.calculate(
            deltaTime.baseUnitMagnitude(),
            new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                current.value.baseUnitMagnitude(), current.slew.baseUnitMagnitude()),
            new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                goal.value.baseUnitMagnitude(), goal.slew.baseUnitMagnitude()));

    Measure<DIM> value = (Measure<DIM>) current.value.baseUnit().of(internalState.position);
    Velocity<DIM> slew = VelocityUnit.combine(value.baseUnit(), Second).of(internalState.velocity);

    // if (Math.abs(current.value().baseUnitMagnitude()) > 10000.0
    //     || Math.abs(goal.value().baseUnitMagnitude()) > 10000.0
    //     || Math.abs(value.baseUnitMagnitude()) > 10000.0) {
    //   System.out.println("yuh");
    // }
    if (MeasureMath.abs(value).gt(maxValue)) {
      value = MeasureMath.clamp(value, maxValue);
      return new State<>(value, (Velocity<DIM>) slew.unit().zero());
    } else {
      return new State<>(value, slew);
    }
  }
}
