package igknighters.subsystems.climber.pivot;

import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.Motors.KrakenX60Foc;

public class PivotConstants {
  public static final double GEAR_RATIO = 50.0;
  public static final int LEADER_MOTOR_ID = 14;
  public static final int FOLLOWER_MOTOR_ID = 15;
  public static final int ENCODER_ID = 26;
  public static final String CANBUS = "rio";
  public static final double KP = 1.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KA = 0.0;
  public static final double KS = 1.0;
  public static final double KV = ((12.0 - KS) / (KrakenX60Foc.FREE_SPEED * Conv.RADIANS_TO_ROTATIONS)) * GEAR_RATIO;
  public static final double FORWARD_LIMIT = Math.PI;
  public static final double REVERSE_LIMIT = -Math.PI;
  public static final boolean INVERT_MOTOR = false;
  public static final double STATOR_CURRENT_LIMIT = 120.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final boolean INVERT_ENCODER = false;
  public static final double ANGLE_OFFSET = 0.0;
  public static final double MAX_VELOCITY = 0.35;
  public static final double MAX_ACCELERATION = 0.5;
}
