package igknighters.constants;

public final class ConstValues {
  private static final double TAU = 2 * Math.PI;

  // all measurements are in meters unless otherwise specified
  // all angles are in radians unless otherwise specified
  public static final class Conv {
    public static final double FEET_TO_METERS = 0.3048;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double METERS_TO_INCHES = 1.0 / INCHES_TO_METERS;
    public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
    public static final double DEGREES_TO_ROTATIONS = 1.0 / 360.0;
    public static final double ROTATIONS_TO_RADIANS = TAU;
    public static final double RPM_TO_RADIANS_PER_SECOND = TAU / 60.0;
    public static final double RADIANS_TO_DEGREES = 180.0 / Math.PI;
    public static final double RADIANS_TO_ROTATIONS = 1.0 / TAU;
    public static final double POUNDS_TO_KILOGRAMS = 0.4535;
  }

  public static final class kMotors {
    public static final class kKrakenX60Foc {
      public static final double FREE_SPEED = 608.0;
      public static final double FREE_CURRENT = 2.0;
      public static final double STALL_TORQUE = 9.37;
      public static final double STALL_CURRENT = 483.0;

      public static final double kV = 12.0 / FREE_SPEED;
    }
  }

  public static final boolean DEBUG = true; // this should be false for competition
  public static final boolean DEMO = false; // this should be false for competition
  public static final double PERIODIC_TIME = 0.02; // 20ms
  public static final int PDH_CAN_ID = 61;

  public static final class kRobotIntrinsics {
    public static final double MASS = 132.0 * Conv.POUNDS_TO_KILOGRAMS;
    public static final double MOMENT_OF_INERTIA = 5.3;

    /** With bumpers */
    public static final double CHASSIS_WIDTH = 32.5 * Conv.INCHES_TO_METERS;

    public static final double MAX_CG = 17.5 * Conv.INCHES_TO_METERS;
    public static final double MIN_CG = 7.0 * Conv.INCHES_TO_METERS;
  }

  public static final class kCharacterization {
    public static final int FREQUENCY = 750;

    public static enum MechanismId {
      SWERVE_ROTATION("Swerve/Rotation"),
      SWERVE_VELOCITY("Swerve/Velocity"),
      SWERVE_STEER("Swerve/Steer");

      public final String name;

      private MechanismId(String name) {
        this.name = name;
      }
    }
  }

  public static final class kLed {
    public static final int LED_COUNT = 38;
    public static final int CANDLE_LEDS = 8;
  }
}
