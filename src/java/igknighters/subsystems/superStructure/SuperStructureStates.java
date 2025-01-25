package igknighters.subsystems.superStructure;

public enum SuperStructureStates {
  ScoreL4(0.0, 0.0),
  ScoreL3(0.0, 0.0),
  ScoreL2(0.0, 0.0),
  ScoreL1(0.0, 0.0);

  public final double elevatorMeters;
  public final double wristRads;


  SuperStructureStates(double elevatorMeters, double wristRads) {
    this.elevatorMeters = elevatorMeters;
    this.wristRads = wristRads;
  }
  
}
