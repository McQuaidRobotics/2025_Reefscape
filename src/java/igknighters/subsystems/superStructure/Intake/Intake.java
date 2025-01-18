package igknighters.subsystems.superStructure.Intake;

import igknighters.subsystems.Component;

public abstract class Intake extends Component {
  public abstract void setTorque(double voltage);

  public abstract boolean hasCoral();

  public abstract boolean hasAlgae();
}
