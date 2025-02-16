package igknighters.subsystems.superStructure.Wrist;

import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.Motors.KrakenX60Foc;

public class WristConstants {
  public static final String CANBUS = "Super";
  public static final int MOTOR_ID = 5;
  public static final int CANCODER_ID = 42;

  public static final boolean INVERT_MOTOR = false;
  public static final boolean INVERT_ENCODER = false;
  public static final double GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (60.0 / 30.0);

  public static final double KP = 18.0;
  public static final double KD = 0.0;
  public static final double KG = 0.0;
  public static final double KS = 0.3;
  public static final double KV =
      (12.0 / (KrakenX60Foc.FREE_SPEED * Conv.RADIANS_TO_ROTATIONS)) * GEAR_RATIO;
  public static final double KA = 0.00;

  public static final double MAX_VELOCITY = (12.0 - KS) / KV;
  public static final double MAX_ACCELERATION = MAX_VELOCITY / 0.5;

  public static final double FORWARD_LIMIT = 87.0 * Conv.DEGREES_TO_ROTATIONS;
  public static final double REVERSE_LIMIT = -87.0 * Conv.DEGREES_TO_ROTATIONS;
  public static final double DEFAULT_TOLERANCE = 1.0 * Conv.DEGREES_TO_ROTATIONS;
  public static final double ANGLE_OFFSET = 0.0;
  public static final double LENGTH = 13.0 * Conv.INCHES_TO_METERS;

  public static final double STATOR_CURRENT_LIMIT = 50.0;
  public static final double SUPPLY_CURRENT_LIMIT = 50.0;
}
