package igknighters.subsystems.superStructure.Elevator;

import igknighters.constants.ConstValues.Conv;

public class ElevatorConstants {
  public static final double ELEVATOR_GEAR_RATIO =
      (Math.PI * 2.0 * 0.01) * (6.0 / 1.0); // IN METERS
  public static final double KP = 0.0;
  public static final double KD = 0.0;
  public static final double ELEVATOR_KG = 0.0;
  public static final double KS = 0.0;
  public static final double KA = 0.0;
  public static final double ELEVATOR_MAX_HEIGHT = Conv.INCHES_TO_METERS * 80.0;
  public static final double Elevator_HEIGHT_ABOVE_GROUND = Conv.INCHES_TO_METERS * 12.8;
  public static final int Elevator_LEADER_ID = 0;
  public static final int Elevator_FOLLOWER_ID = 1;
}
