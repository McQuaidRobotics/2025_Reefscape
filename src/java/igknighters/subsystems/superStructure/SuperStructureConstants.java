package igknighters.subsystems.superStructure;

import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kMotors.kKrakenX60Foc;

public class SuperStructureConstants {
  public static final String CANBUS = "Super";

  public class kElevator {
    /** Wire run side */
    public static final int LEADER_ID = 9;

    /** Opposite wire run side */
    public static final int FOLLOWER_ID = 10;

    public static final int LIMIT_SWITCH_ID = 9;

    public static final boolean INVERT_LEADER = false;
    public static final double GEAR_RATIO = 6.0;
    // public static final double GEAR_RATIO = (54.0 / 18.0) * (54.0 / 18.0);
    public static final double PULLEY_RADIUS = Conv.INCHES_TO_METERS * 1.106;
    public static final double PULLEY_CIRCUMFERENCE = 2.0 * Math.PI * PULLEY_RADIUS;

    public record Stage(double tubeLength, double mass) {
      public static final double OVERLAP = 5.0 * Conv.INCHES_TO_METERS;
      public static final double FIRST_STAGE_HEIGHT = 32.0 * Conv.INCHES_TO_METERS;
      public static final double TUBE_HEIGHT = 1.0 * Conv.INCHES_TO_METERS;

      public double rangeOfMotion() {
        return tubeLength - OVERLAP - TUBE_HEIGHT;
      }
    }

    public static final Stage[] STAGES = {
      new Stage(Stage.FIRST_STAGE_HEIGHT, 2.125),
      new Stage(Stage.FIRST_STAGE_HEIGHT - Stage.TUBE_HEIGHT, 2.125),
      new Stage(Stage.FIRST_STAGE_HEIGHT - Stage.TUBE_HEIGHT * 3.0, 2.125),
      new Stage(Conv.INCHES_TO_METERS * 8.0, 4.5)
    };
    public static final double TOTAL_RANGE_OF_MOTION =
        STAGES[0].rangeOfMotion()
            + STAGES[1].rangeOfMotion()
            + STAGES[2].rangeOfMotion()
            + STAGES[3].rangeOfMotion();

    public static final double MOI =
        (PULLEY_RADIUS * PULLEY_RADIUS)
            * (STAGES[0].mass + STAGES[1].mass + STAGES[2].mass + STAGES[3].mass);

    public static final double MIN_HEIGHT = 13.1 * Conv.INCHES_TO_METERS;
    public static final double MAX_HEIGHT = 80.5 * Conv.INCHES_TO_METERS;
    public static final double DEFAULT_TOLERANCE = 0.75 * Conv.INCHES_TO_METERS;

    public static final double kP = 6.0;
    public static final double kD = 0.0;
    public static final double kG = 0.05;
    public static final double kS = 0.3;
    public static final double kV = kKrakenX60Foc.kV * GEAR_RATIO;
    public static final double kA = 0.00;

    public static final double MAX_VELOCITY = (12.0 - kS - kG) / kV;
    public static final double MAX_ACCELERATION = MAX_VELOCITY / 0.135;

    public static final double MAX_VELOCITY_LINEAR = MAX_VELOCITY * PULLEY_RADIUS;
    public static final double MAX_ACCELERATION_LINEAR = MAX_ACCELERATION * PULLEY_RADIUS;


    public static final double HOMING_VOLTAGE = -kS - 2.0;

    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  }

  public class kWrist {
    public static final double LENGTH = 9.0 * Conv.INCHES_TO_METERS;
    public static final double MAX_ANGLE = -0.20 * Conv.ROTATIONS_TO_RADIANS;
    public static final double ALGAE_MAX_ANGLE = -0.07 * Conv.ROTATIONS_TO_RADIANS;
    public static final double MIN_ANGLE = 0.127 * Conv.ROTATIONS_TO_RADIANS;

    public static final int MOTOR_ID = 11;
    public static final int CANCODER_ID = 11;

    public static final boolean INVERT_MOTOR = true;
    public static final boolean INVERT_ENCODER = false;
    public static final double GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (60.0 / 30.0);

    public static final double KP = 110.0;
    public static final double KD = 0.0;
    public static final double KG = 0.07;
    public static final double KS = 0.2;
    public static final double KV = kKrakenX60Foc.kV * GEAR_RATIO;
    public static final double KA = 0.00;

    public static final double MAX_VELOCITY = (12.0 - KS - KG) / KV;
    public static final double MAX_ACCELERATION = MAX_VELOCITY / 0.1;
    public static final double ALGAE_MAX_VELOCITY = MAX_VELOCITY * 0.75;
    public static final double ALGAE_MAX_ACCELERATION = MAX_ACCELERATION * 0.5;

    public static final double DEFAULT_TOLERANCE = 1.3 * Conv.DEGREES_TO_RADIANS;
    public static final double ANGLE_OFFSET = -0.30810546875;

    public static final double STATOR_CURRENT_LIMIT = 50.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double WRIST_AXEL_LOWER_LIMIT =
        COLLISION_HEIGHT - (kWrist.LENGTH * Math.sin(kWrist.MAX_ANGLE));
  }

  public static final double COLLISION_HEIGHT = 8.0 * Conv.INCHES_TO_METERS;
}
