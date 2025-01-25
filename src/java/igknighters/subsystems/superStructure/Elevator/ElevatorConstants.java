package igknighters.subsystems.superStructure.Elevator;

import igknighters.constants.ConstValues.Conv;

public class ElevatorConstants {
  public static final double GEAR_RATIO = (Math.PI * 2.0) * (6.0 / 1.0);
  public static final double KP = 0.0;
  public static final double KD = 0.0;
  public static final double KG = 0.0;
  public static final double KS = 0.0;
  public static final double KA = 0.0;
  public static final double MAX_HEIGHT = Conv.INCHES_TO_METERS * 80.0;
  public static final double HEIGHT_ABOVE_GROUND = Conv.INCHES_TO_METERS * 12.8;
  public static final int LEADER_ID = 0;
  public static final int FOLLOWER_ID = 1;
  public static final double MAX_ACCELERATION = 4.0;
  public static final double MAX_VELOCITY = 2.0;
  public static final double WHEEL_RADIUS = 0.085; // METERS
}
