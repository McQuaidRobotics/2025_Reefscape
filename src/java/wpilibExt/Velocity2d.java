package wpilibExt;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.util.Objects;

public class Velocity2d implements Interpolatable<Velocity2d>, StructSerializable {
  private final double m_vx;
  private final double m_vy;

  /** Constructs a Velocity2d with X and Y components equal to zero. */
  public Velocity2d() {
    this(0.0, 0.0);
  }

  /**
   * Constructs a Velocity2d with the X and Y components equal to the provided values.
   *
   * @param vx The x component of the velocity.
   * @param vy The y component of the velocity.
   */
  public Velocity2d(double vx, double vy) {
    m_vx = vx;
    m_vy = vy;
  }

  /**
   * Constructs a Velocity2d with the provided speed and angle. This is essentially converting from
   * polar coordinates to Cartesian coordinates.
   *
   * @param speed The speed of the velocity.
   * @param angle The angle between the x-axis and the velocity vector.
   */
  public Velocity2d(double speed, Rotation2d angle) {
    this(speed * angle.getCos(), speed * angle.getSin());
  }

  /**
   * Constructs a Velocity2d with the X and Y components equal to the provided values. The X and Y
   * components will be converted to and tracked as meters per second.
   *
   * @param x The x component of the velocity.
   * @param y The y component of the velocity.
   */
  public Velocity2d(LinearVelocity x, LinearVelocity y) {
    this(x.in(MetersPerSecond), y.in(MetersPerSecond));
  }

  /**
   * Constructs a Velocity2d from the provided velocity vector's X and Y components. The values are
   * assumed to be in meters per second.
   *
   * @param vector The velocity vector to represent.
   */
  public Velocity2d(Vector<N2> vector) {
    this(vector.get(0), vector.get(1));
  }

  /**
   * Constructs a Velocity2d from the xy components of the provided {@link ChassisSpeeds}.
   *
   * @param chassisSpeeds The chassis speeds to extract the velocity from.
   * @return The velocity of the chassis.
   */
  public Velocity2d(ChassisSpeeds chassisSpeeds) {
    this(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  /**
   * Returns the X component of the velocity.
   *
   * @return The X component of the velocity.
   */
  @JsonProperty
  public double getVX() {
    return m_vx;
  }

  /**
   * Returns the Y component of the velocity.
   *
   * @return The Y component of the velocity.
   */
  @JsonProperty
  public double getVY() {
    return m_vy;
  }

  /**
   * Returns a vector representation of this velocity.
   *
   * @return A Vector representation of this velocity.
   */
  public Vector<N2> toVector() {
    return VecBuilder.fill(m_vx, m_vy);
  }

  /**
   * Returns the speed of the velocity.
   *
   * @return The speed of the velocity.
   */
  public double getSpeed() {
    return Math.hypot(m_vx, m_vy);
  }

  public double speedInDirection(Rotation2d direction) {
    if (MathUtil.isNear(0, getSpeed(), 0.0001)) {
      return 0;
    }
    return m_vx * direction.getCos() + m_vy * direction.getSin();
  }

  /**
   * Returns the angle this velocity forms with the positive X axis.
   *
   * @return The angle of the velocity
   */
  public Rotation2d getDirection() {
    return new Rotation2d(m_vx, m_vy);
  }

  /**
   * Applies a rotation to the velocity in 2D space.
   *
   * <p>This multiplies the velocity vector by a counterclockwise rotation matrix of the given
   * angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * <p>For example, rotating a Velocity2d of &lt;2, 0&gt; by 90 degrees will return a Velocity2d of
   * &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   * @return The new rotated translation.
   */
  public Velocity2d rotateBy(Rotation2d other) {
    return new Velocity2d(
        m_vx * other.getCos() - m_vy * other.getSin(),
        m_vx * other.getSin() + m_vy * other.getCos());
  }

  /**
   * Returns the sum of two velocitys in 2D space.
   *
   * <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d{3.0, 8.0).
   *
   * @param other The velocity to add.
   * @return The sum of the velocitys.
   */
  public Velocity2d plus(Velocity2d other) {
    return new Velocity2d(m_vx + other.m_vx, m_vy + other.m_vy);
  }

  /**
   * Returns the difference between two velocitys.
   *
   * <p>For example, Velocity2d(5.0, 4.0) - Velocity2d(1.0, 2.0) = Velocity2d(4.0, 2.0).
   *
   * @param other The velocity to subtract.
   * @return The difference between the two velocitys.
   */
  public Velocity2d minus(Velocity2d other) {
    return new Velocity2d(m_vx - other.m_vx, m_vy - other.m_vy);
  }

  /**
   * Returns the inverse of the current velocity. This is equivalent to rotating by 180 degrees,
   * flipping the point over both axes, or negating all components of the velocity.
   *
   * @return The inverse of the current velocity.
   */
  public Velocity2d unaryMinus() {
    return new Velocity2d(-m_vx, -m_vy);
  }

  /**
   * Returns the velocity multiplied by a scalar.
   *
   * <p>For example, Velocity2d(2.0, 2.5) * 2 = Velocity2d(4.0, 5.0).
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled velocity.
   */
  public Velocity2d times(double scalar) {
    return new Velocity2d(m_vx * scalar, m_vy * scalar);
  }

  /**
   * Returns the velocity divided by a scalar.
   *
   * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
   *
   * @param scalar The scalar to multiply by.
   * @return The reference to the new mutated object.
   */
  public Velocity2d div(double scalar) {
    return new Velocity2d(m_vx / scalar, m_vy / scalar);
  }

  @Override
  public String toString() {
    return String.format("Velocity2d(X: %.2f, Y: %.2f)", m_vx, m_vy);
  }

  /**
   * Checks equality between this Velocity2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Velocity2d) {
      return Math.abs(((Velocity2d) obj).m_vx - m_vx) < 1E-9
          && Math.abs(((Velocity2d) obj).m_vy - m_vy) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_vx, m_vy);
  }

  @Override
  public Velocity2d interpolate(Velocity2d endValue, double t) {
    return new Velocity2d(
        MathUtil.interpolate(this.getVX(), endValue.getVX(), t),
        MathUtil.interpolate(this.getVY(), endValue.getVY(), t));
  }

  public static final Velocity2dStruct struct = new Velocity2dStruct();

  public static class Velocity2dStruct implements Struct<Velocity2d> {
    @Override
    public Class<Velocity2d> getTypeClass() {
      return Velocity2d.class;
    }

    @Override
    public String getTypeName() {
      return "Velocity2d";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
      return "double x;double y";
    }

    @Override
    public Velocity2d unpack(ByteBuffer bb) {
      double vx = bb.getDouble();
      double vy = bb.getDouble();
      return new Velocity2d(vx, vy);
    }

    @Override
    public void pack(ByteBuffer bb, Velocity2d value) {
      bb.putDouble(value.getVX());
      bb.putDouble(value.getVY());
    }

    @Override
    public boolean isImmutable() {
      return true;
    }
  }

  public static final Velocity2d kZero = new Velocity2d(0.0, 0.0);
}
