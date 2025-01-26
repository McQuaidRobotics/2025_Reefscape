package igknighters.subsystems.superStructure.Elevator;

import igknighters.constants.ConstValues.Conv;

public class ElevatorConstants {
  public static final String CANBUS = "Superstructure";
  public static final int LEADER_ID = 11;
  public static final int FOLLOWER_ID = 12;

  public static final double GEAR_RATIO = (54.0 / 18.0) * (54.0 / 18.0);
  public static final double WHEEL_RADIUS = Conv.INCHES_TO_METERS * 1.106;
  public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;

  public static final double KP = 1.0;
  public static final double KD = 0.0;
  public static final double KG = 0.0;
  public static final double KS = 0.0;
  public static final double KA = 0.0;

  public static final double MAX_ACCELERATION = 12.0;
  public static final double MAX_VELOCITY = 4.0;

  public static final double MAX_HEIGHT = Conv.INCHES_TO_METERS * 80.0;
  public static final double MIN_HEIGHT = Conv.INCHES_TO_METERS * 12.75;
  public static final double TOLERANCE = 0.02;
}
