package igknighters.subsystems.swerve.odometryThread;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record SwerveDriveSample(
    SwerveModuleSuperState[] modulePositions,
    Rotation2d gyroYaw,
    double acceleration,
    double timestamp)
    implements StructSerializable, Cloneable {

  public static class SwerveModuleSuperState extends SwerveModulePosition {
    public double speedMetersPerSecond;

    public SwerveModuleSuperState(
        double distanceMeters, double velocityMetersPerSecond, Rotation2d angle) {
      super(distanceMeters, angle);
      this.speedMetersPerSecond = velocityMetersPerSecond;
    }

    public SwerveModuleSuperState(SwerveModulePosition position, double velocityMetersPerSecond) {
      super(position.distanceMeters, position.angle);
      this.speedMetersPerSecond = velocityMetersPerSecond;
    }
  }

  public SwerveDriveSample clone() {
    final SwerveModuleSuperState[] newModulePositions =
        new SwerveModuleSuperState[this.modulePositions.length];
    for (int i = 0; i < modulePositions.length; i++) {
      final SwerveModuleSuperState oldModulePosition = this.modulePositions[i];
      newModulePositions[i] =
          new SwerveModuleSuperState(
              oldModulePosition.distanceMeters,
              oldModulePosition.speedMetersPerSecond,
              oldModulePosition.angle);
    }
    return new SwerveDriveSample(
        newModulePositions,
        new Rotation2d(gyroYaw.getCos(), gyroYaw.getSin()),
        acceleration,
        timestamp);
  }

  public static final Struct<SwerveDriveSample> struct = new SwerveDriveSampleStruct();

  public static class SwerveDriveSampleStruct implements Struct<SwerveDriveSample> {
    @Override
    public Class<SwerveDriveSample> getTypeClass() {
      return SwerveDriveSample.class;
    }

    @Override
    public int getSize() {
      return SwerveModulePosition.struct.getSize() * 4
          + Rotation2d.struct.getSize()
          + Double.BYTES * 2;
    }

    @Override
    public String getTypeName() {
      return "SwerveDriveSample";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {SwerveModulePosition.struct, Rotation2d.struct};
    }

    @Override
    public boolean isImmutable() {
      return true;
    }

    @Override
    public SwerveDriveSample clone(SwerveDriveSample obj) throws CloneNotSupportedException {
      return obj.clone();
    }

    @Override
    public boolean isCloneable() {
      return true;
    }

    @Override
    public String getSchema() {
      return "Rotation2d gyroYaw;"
          + "double gforce;"
          + "double timestamp;"
          + "SwerveModulePosition module0;"
          + "SwerveModulePosition module1;"
          + "SwerveModulePosition module2;"
          + "SwerveModulePosition module3;";
    }

    @Override
    public void pack(ByteBuffer bb, SwerveDriveSample value) {
      Rotation2d.struct.pack(bb, value.gyroYaw);
      bb.putDouble(value.acceleration);
      bb.putDouble(value.timestamp);
      for (SwerveModulePosition module : value.modulePositions) {
        SwerveModulePosition.struct.pack(bb, module);
      }
    }

    @Override
    public SwerveDriveSample unpack(ByteBuffer bb) {
      throw new UnsupportedOperationException("Not implemented");
    }
  }
}
