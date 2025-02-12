package igknighters.subsystems.climber.electroMagnet;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class ElectroMagnet extends Component {
  @Log boolean isOn = false;

  public abstract void setOn(boolean on);
}
