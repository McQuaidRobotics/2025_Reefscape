package igknighters.subsystems.climber.electroMagnet;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class ElectroMagnetReal extends ElectroMagnet {
  private final Relay electromagnet = new Relay(ElectroMagnetConstants.RELAY_PORT);

  public void setOn(boolean on) {
    // TODO Auto-generated method stub
    if (on) {
      super.isOn = true;
      electromagnet.set(Value.kOn);
    } else {
      super.isOn = false;
      electromagnet.set(Value.kOff);
    }
  }
}
