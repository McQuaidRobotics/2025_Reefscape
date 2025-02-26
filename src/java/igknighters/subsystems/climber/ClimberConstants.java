package igknighters.subsystems.climber;

import igknighters.constants.ConstValues.Conv;

public class ClimberConstants {
  public static final String CANBUS = "rio";

  public class PivotConstants {
    public static final double GEAR_RATIO = 50.0;
    public static final int LEADER_MOTOR_ID = 14;
    public static final int FOLLOWER_MOTOR_ID = 15;
    public static final int ENCODER_ID = LEADER_MOTOR_ID;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double FORWARD_LIMIT = 0.378174 * Conv.ROTATIONS_TO_RADIANS;
    public static final double REVERSE_LIMIT = -0.373047 * Conv.ROTATIONS_TO_RADIANS;
    public static final boolean INVERT_MOTOR = false;

    public static final double STATOR_CURRENT_LIMIT = 120.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final boolean INVERT_ENCODER = false;

    public static final double MAX_VELOCITY = 0.35;
    public static final double MAX_ACCELERATION = 0.5;

    public static final double ANGLE_OFFSET = 0.03271484375;
  }

  public class ElectroMagnetConstants {
    public static final int RELAY_PORT = 0;
  }
}
