package igknighters.subsystems.Intake.rollers;

import igknighters.subsystems.Component;

public abstract class Rollers extends Component {
  public abstract void setCurrent(double voltage);

  public abstract boolean hasCoral();

  public abstract boolean hasAlgae();
}
