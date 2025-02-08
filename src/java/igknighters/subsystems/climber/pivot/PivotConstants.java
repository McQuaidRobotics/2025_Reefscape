package igknighters.subsystems.climber.pivot;

public class PivotConstants{
  public static final double GEAR_RATIO = 50.0;
  public static final int MOTOR_ID = 0;  
  public static final int ENCODER_ID = 40;
  public static final String MOTOR_CANBUS = "drivebus";
  public static final String ENCODER_CANBUS = "drivebus";
  public static final double KP = 1.0;
  public static final double KI = 1.0;
  public static final double KD = 1.0;
  public static final double KA = 1.0;
  public static final double KS = 1.0;
  public static final double KV = 1.0;
  public static final double KG = 1.0;
  public static final double FORWARD_LIMIT = Math.PI;
  public static final double REVERSE_LIMIT = -Math.PI;
  public static final double STATOR_CURRENT_LIMIT = 40.0;
  public static final boolean INVERT_MOTOR = false;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final boolean INVERT_ENCODER = false;
  public static final double ANGLE_OFFSET = 0.0;
  public static final double MAX_VELOCITY = 1.0;
  public static final double MAX_ACCELERATION = 1.0;

}
