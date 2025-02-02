package igknighters.subsystems.superStructure.Elevator;

import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.Motors.KrakenX60Foc;

public class ElevatorConstants {
  public static final String CANBUS = "Superstructure";
  public static final int LEADER_ID = 11;
  public static final int FOLLOWER_ID = 12;

  public static final double GEAR_RATIO = (54.0 / 18.0) * (54.0 / 18.0);
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

  public static final double MOI =
      (PULLEY_RADIUS * PULLEY_RADIUS)
          * (STAGES[0].mass + STAGES[1].mass + STAGES[2].mass + STAGES[3].mass);

  // all units below are in radians
  public static final double KP = 15.0;
  public static final double KD = 0.5;
  public static final double KG = 0.0;
  public static final double KS = 0.3;
  public static final double KV = (12.0 / (KrakenX60Foc.FREE_SPEED)) * GEAR_RATIO;
  public static final double KA = 0.0;

  public static final double MAX_VELOCITY = (12.0 - KS - KG) / KV;
  public static final double MAX_ACCELERATION = MAX_VELOCITY / 0.2;

  // is actually .75 inches more
  public static final double FORWARD_LIMIT = (81.0 * Conv.INCHES_TO_METERS) / PULLEY_RADIUS;
  public static final double REVERSE_LIMIT = (12.75 * Conv.INCHES_TO_METERS) / PULLEY_RADIUS;
  public static final double DEFAULT_TOLERANCE = (1.0 * Conv.INCHES_TO_METERS) / PULLEY_RADIUS;

  public static final double HOMING_VOLTAGE = -KS * 1.5;

  public static final double STATOR_CURRENT_LIMIT = 80.0;
  public static final double SUPPLY_CURRENT_LIMIT = 50.0;
}
