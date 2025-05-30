package wpilibExt;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Mult;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.struct.Struct;
import java.util.function.BiFunction;
import monologue.ProceduralStructGenerator;

public class MeasureMath {

  public static final class Constants {
    public static final Angle kPi = Radian.of(Math.PI);
    public static final Angle kCCW_2Pi = Radian.of(2.0 * Math.PI);
    public static final Angle kCW_2Pi = Radian.of(-2.0 * Math.PI);

    public static final LinearAcceleration kGravity = MetersPerSecondPerSecond.of(9.8);
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M abs(M m) {
    return (M) m.unit().ofBaseUnits(Math.abs(m.baseUnitMagnitude()));
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M negate(M m) {
    return (M) m.unit().ofBaseUnits(-m.baseUnitMagnitude());
  }

  public static <U extends Unit, M extends Measure<U>> M max(M m1, M m2) {
    return m1.baseUnitMagnitude() > m2.baseUnitMagnitude() ? m1 : m2;
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M max(M m1, M m2, M... otherMs) {
    M max = max(m1, m2);
    for (M m : otherMs) {
      max = max(max, m);
    }
    return max;
  }

  public static <U extends Unit, M extends Measure<U>> M maxAbs(M m1, M m2) {
    return abs(m1).baseUnitMagnitude() > abs(m2).baseUnitMagnitude() ? m1 : m2;
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M maxAbs(M m1, M m2, M... otherMs) {
    M max = maxAbs(m1, m2);
    for (M m : otherMs) {
      max = max(max, m);
    }
    return max;
  }

  public static <U extends Unit, M extends Measure<U>> M min(M m1, M m2) {
    return m1.baseUnitMagnitude() < m2.baseUnitMagnitude() ? m1 : m2;
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M min(M m1, M m2, M... otherMs) {
    M min = min(m1, m2);
    for (M m : otherMs) {
      min = min(min, m);
    }
    return min;
  }

  public static <U extends Unit, M extends Measure<U>> M minAbs(M m1, M m2) {
    return abs(m1).baseUnitMagnitude() < abs(m2).baseUnitMagnitude() ? m1 : m2;
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M minAbs(M m1, M m2, M... otherMs) {
    M min = minAbs(m1, m2);
    for (M m : otherMs) {
      min = min(min, m);
    }
    return min;
  }

  public static <U extends Unit, M extends Measure<U>> M clamp(M m, M min, M max) {
    if (min.gt(max)) {
      throw new IllegalArgumentException("min must be less than or equal to max");
    }
    return max(min, min(max, m));
  }

  public static <U extends Unit, M extends Measure<U>> M clamp(M m, M magnitudeLimit) {
    return max(negate(abs(magnitudeLimit)), min(abs(magnitudeLimit), m));
  }

  public static <U extends Unit, M extends Measure<U>> double signum(M m) {
    return Math.signum(m.baseUnitMagnitude());
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M nudgeZero(M m, M tolerance) {
    return abs(m).baseUnitMagnitude() < tolerance.baseUnitMagnitude() ? (M) m.unit().zero() : m;
  }

  public record XY<M extends Measure<?>>(M x, M y) {
    public static XY<Distance> of(Translation2d t) {
      return new XY<>(Meters.of(t.getX()), Meters.of(t.getY()));
    }

    public static <M extends Measure<?>> XY<M> of(M x, M y) {
      return new XY<>(x, y);
    }

    @SuppressWarnings("unchecked")
    public static <M extends Measure<?>> XY<M> of(M mag, Rotation2d rot) {
      return new XY<>((M) mag.times(rot.getCos()), (M) mag.times(rot.getSin()));
    }

    @SuppressWarnings("unchecked")
    public static <U extends Unit> XY<Measure<U>> zero(U unit) {
      return new XY<>((Measure<U>) unit.zero(), (Measure<U>) unit.zero());
    }

    public <UN extends Unit, N extends Measure<UN>, R extends Measure<?>> N cross(
        XY<R> rhs, Class<N> cls, BiFunction<M, R, N> f) {
      var a = f.apply(x, rhs.y);
      var b = f.apply(y, rhs.x);
      return cls.cast(a.minus(b));
    }

    @SuppressWarnings("unchecked")
    public M magnitude() {
      return (M) x.unit().ofBaseUnits(Math.hypot(x.baseUnitMagnitude(), y.baseUnitMagnitude()));
    }

    @SuppressWarnings("unchecked")
    public XY<M> normalize() {
      M mag = magnitude();
      return new XY<>((M) x.div(mag), (M) y.div(mag));
    }

    @SuppressWarnings("unchecked")
    public XY<M> times(M scalar) {
      return new XY<>((M) x.times(scalar), (M) y.times(scalar));
    }

    public XY<M> desaturate(M maxMagnitude) {
      if (magnitude().baseUnitMagnitude() <= maxMagnitude.baseUnitMagnitude()) {
        return this;
      }
      return normalize().times(maxMagnitude);
    }

    @SuppressWarnings("rawtypes")
    public static final Struct<XY> struct = ProceduralStructGenerator.genRecord(XY.class);
  }

  public static Torque times(Distance d, Force f) {
    return NewtonMeters.of(d.in(Meters) * f.in(Newtons));
  }

  public static Torque times(Force f, Distance d) {
    return NewtonMeters.of(d.in(Meters) * f.in(Newtons));
  }

  // https://openstax.org/books/university-physics-volume-1/pages/10-7-newtons-second-law-for-rotation
  public static AngularAcceleration div(Torque t, MomentOfInertia moi) {
    return RadiansPerSecondPerSecond.of(t.in(NewtonMeters) / moi.in(KilogramSquareMeters));
  }

  public static Force times(LinearAcceleration a, Mass m) {
    return Newtons.of(m.in(Kilograms) * a.in(MetersPerSecondPerSecond));
  }

  public static Torque times(AngularAcceleration a, MomentOfInertia moi) {
    return NewtonMeters.of(moi.in(KilogramSquareMeters) * a.in(RadiansPerSecondPerSecond));
  }

  public static LinearAcceleration times(AngularAcceleration a, Distance d) {
    return MetersPerSecondPerSecond.of(a.in(RadiansPerSecondPerSecond) * d.in(Meters));
  }

  public static MomentOfInertia times(Mass m, Mult<DistanceUnit, DistanceUnit> d) {
    return KilogramSquareMeters.of(m.in(Kilograms) * d.baseUnitMagnitude());
  }

  public static Mass div(MomentOfInertia moi, Mult<DistanceUnit, DistanceUnit> d) {
    return Kilograms.of(moi.in(KilogramSquareMeters) / d.baseUnitMagnitude());
  }

  public static Velocity<AngleUnit> genericize(AngularVelocity av) {
    return VelocityUnit.combine(Radian, Second).of(av.in(RadiansPerSecond));
  }

  public static Acceleration<AngleUnit> genericize(AngularAcceleration aa) {
    return AccelerationUnit.combine(VelocityUnit.combine(Radian, Second), Second)
        .of(aa.in(RadiansPerSecondPerSecond));
  }

  @SuppressWarnings("unchecked")
  public static <M extends Measure<?>> M zeroIfNAN(M m) {
    if (Double.isNaN(m.baseUnitMagnitude())) {
      return (M) m.unit().zero();
    }
    return m;
  }
}
