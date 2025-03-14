package igknighters.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kMotors.kKrakenX60Foc;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kDriveMotor;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kWheel;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;

public class SwerveConstants {

  /** The gear ratios for the swerve modules for easier constant definition. */
  @SuppressWarnings("unused")
  private static final class SwerveGearRatios {
    static final double L2_DRIVE = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    static final double L2_DRIVE_KRAKEN = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    static final double L3_DRIVE = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    static final double L3_DRIVE_KRAKEN = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);

    static final double STEER = 150.0 / 7.0;
  }

  public static final class kGyro {
    public static final int PIGEON_ID = 33;
    public static final boolean INVERT = false;
  }

  public static final class ModuleConstants {
    public static final class kWheel {
      public static final double RADIUS = 1.92 * Conv.INCHES_TO_METERS;
      public static final double DIAMETER = RADIUS * 2.0;
      public static final double CIRCUMFERENCE = DIAMETER * Math.PI;
      public static final double COF = 1.8;
    }

    public static final class kDriveMotor {
      public static final double GEAR_RATIO = SwerveGearRatios.L3_DRIVE_KRAKEN;

      public static final boolean INVERT = false;
      public static final boolean NEUTRAL_MODE_BRAKE = false;

      public static final double kP = 0.3;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kS = 0.15;
      public static final double kV = kKrakenX60Foc.kV * GEAR_RATIO;

      public static final double STATOR_CURRENT_LIMIT = 120.0;
      public static final double SUPPLY_CURRENT_LIMIT = 50.0;

      public static final double MAX_VELOCITY = (12.0 - kS) / kV;
    }

    public static final class kSteerMotor {
      public static final double GEAR_RATIO = SwerveGearRatios.STEER;

      public static final boolean INVERT = true;
      public static final boolean NEUTRAL_MODE_BRAKE = true;

      public static final double kP = 30.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kS = 0.0;
      public static final double kV = kKrakenX60Foc.kV * GEAR_RATIO;

      public static final double STATOR_CURRENT_LIMIT = 60.0;
      public static final double SUPPLY_CURRENT_LIMIT = 20.0;

      public static final double MAX_VELOCITY = (12.0 - kS) / kV;
    }

    public static final class kSteerEncoder {
      public static final boolean INVERT = false;

      public static final double[] ENCODER_OFFSETS_ROTATIONS =
          new double[] {-0.2055, -0.39453, 0.35766, -0.192383};
    }
  }

  public static final class kSwerve {
    public static final String CANBUS = "Drive";

    public static final double DRIVEBASE_WIDTH = 20.75 * Conv.INCHES_TO_METERS;
    public static final double DRIVEBASE_RADIUS =
        Math.hypot(DRIVEBASE_WIDTH / 2.0, DRIVEBASE_WIDTH / 2.0);

    public static final double MAX_DRIVE_VELOCITY = kDriveMotor.MAX_VELOCITY * kWheel.RADIUS;
    public static final double MAX_DRIVE_ACCELERATION = MAX_DRIVE_VELOCITY / 0.8;
    public static final double MAX_ANGULAR_VELOCITY = (MAX_DRIVE_VELOCITY / DRIVEBASE_RADIUS);
    public static final double MAX_STEERING_VELOCITY = ModuleConstants.kSteerMotor.MAX_VELOCITY;

    public static final LerpTable TELEOP_TRANSLATION_AXIS_CURVE =
        new LerpTable(
            new LerpTableEntry(0.0, 0.0),
            new LerpTableEntry(0.05, 0.0), // deadzone
            new LerpTableEntry(0.7, 0.55),
            new LerpTableEntry(1.0, 1.0));

    public static final LerpTable TELEOP_ROTATION_AXIS_CURVE =
        new LerpTable(
            new LerpTableEntry(0.0, 0.0),
            new LerpTableEntry(0.05, 0.0), // deadzone
            new LerpTableEntry(0.5, 0.3),
            new LerpTableEntry(0.7, 0.6),
            new LerpTableEntry(1.0, 1.0));

    public static final Translation2d[] MODULE_CHASSIS_LOCATIONS =
        new Translation2d[] {
          new Translation2d(DRIVEBASE_WIDTH / 2.0, -DRIVEBASE_WIDTH / 2.0),
          new Translation2d(-DRIVEBASE_WIDTH / 2.0, -DRIVEBASE_WIDTH / 2.0),
          new Translation2d(-DRIVEBASE_WIDTH / 2.0, DRIVEBASE_WIDTH / 2.0),
          new Translation2d(DRIVEBASE_WIDTH / 2.0, DRIVEBASE_WIDTH / 2.0)
        };

    public static final ChassisConstraints CONSTRAINTS =
        new ChassisConstraints(
            new Constraints(MAX_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY * 3.0),
            new Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY));
  }
}
