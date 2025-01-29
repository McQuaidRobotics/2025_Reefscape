package igknighters.subsystems.superStructure.Wrist;

import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.Conv;

public class WristConstants {
  public static final int ID = 5;
  public static final double KP = 0.0;
  public static final double KD = 0.0;
  public static final double KG = 0.0;
  public static final double KS = 0.75;
  public static final double KV = 9.5;
  public static final double KA = 0.15;
  public static final double GEAR_RATIO = 40.0;
  public static final double MAX_VELOCITY = ConstValues.Motors.KrakenX60Foc.FREE_SPEED / GEAR_RATIO;
  public static final double MAX_ACCELERATION = MAX_VELOCITY / 0.5;
  public static final double MAX_ANGLE = 87.0 * Conv.DEGREES_TO_RADIANS;
  public static final double MIN_ANGLE = -87.0 * Conv.DEGREES_TO_RADIANS;
  public static final int CANCODER_ID = 42;
  public static final double ANGLE_OFFSET = 0.0;
  public static final double TOLERANCE = 1.0 * Conv.DEGREES_TO_RADIANS;
}
