package igknighters.subsystems.climber.electroMagnet;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import igknighters.subsystems.climber.ClimberConstants.ElectroMagnetConstants;

public class ElectroMagnetReal extends ElectroMagnet {
  private final Relay electromagnet = new Relay(ElectroMagnetConstants.RELAY_PORT);

  public void setOn(boolean on) {
    if (on) {
      super.isOn = true;
      electromagnet.set(Value.kForward);
    } else {
      super.isOn = false;
      electromagnet.set(Value.kOff);
    }
  }
}
