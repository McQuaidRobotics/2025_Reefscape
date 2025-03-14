package igknighters.commands.autos;

public enum Waypoints {
  CloseLeft_R,
  CloseLeft_M,
  CloseLeft_L,
  CloseRight_R,
  CloseRight_M,
  CloseRight_L,
  CloseMid_R,
  CloseMid_L,
  CloseMid_M,
  FarLeft_R,
  FarLeft_M,
  FarLeft_L,
  FarRight_R,
  FarRight_M,
  FarRight_L,
  FarMid_R,
  FarMid_L,
  FarMid_M,
  Intake,
  StartingInside,
  StartingOutside;

  public String to(Waypoints wp) {
    return this.name() + "To" + wp.name();
  }
}
