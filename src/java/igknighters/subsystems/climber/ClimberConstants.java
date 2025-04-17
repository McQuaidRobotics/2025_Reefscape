package igknighters.subsystems.climber;

import igknighters.constants.ConstValues.Conv;

public class ClimberConstants {
  public static final String CANBUS = "rio";

  public class kPivot {
    public static final double GEAR_RATIO = (5.0 * 5.0 * 5.0) * (30.0 / 12.0);
    public static final int LEADER_MOTOR_ID = 14;
    public static final int FOLLOWER_MOTOR_ID = 15;
    public static final int ENCODER_ID = LEADER_MOTOR_ID;
    public static final double KP = 200.0;
    public static final double KI = 10.0;
    public static final double KD = 5.0;
    public static final boolean INVERT_MOTOR = false;

    public static final double STATOR_CURRENT_LIMIT = 120.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final boolean INVERT_ENCODER = true;

    public static final double STAGE_ANGLE = 20.0 * Conv.DEGREES_TO_RADIANS;
    public static final double ASCEND_ANGLE = -125.0 * Conv.DEGREES_TO_RADIANS;
    public static final double STOW_ANGLE = -155.0 * Conv.DEGREES_TO_RADIANS;

    public static final double ANGLE_OFFSET = -0.03271484375;
  }
}
