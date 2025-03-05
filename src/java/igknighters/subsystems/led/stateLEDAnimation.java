package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum stateLEDAnimation {
  DISABLED(LEDPattern.solid(new Color(255, 0, 0)), 50, 0, 0, false),
  TELEOP(LEDPattern.solid(new Color(0, 0, 255)), 50, 0, 0, false),
  AUTO(LEDPattern.rainbow(255, 100), 50, 0, 1, false),
  READYTOPLACE(LEDPattern.solid(new Color(0, 255, 0)), 10, 40, 0, false),
  AUTOALINING(LEDPattern.solid(new Color(255, 0, 255)), 40, 0, 0, false),
  PLACING(LEDPattern.solid(new Color(255, 255, 0)), 50, 0, 0, false);
  
  public final LEDPattern pattern;
  public final int length;
  public final int offset;
  public final double speed;
  public final boolean reversed;

  private stateLEDAnimation(LEDPattern pattern, int length, int offset, double speed, boolean reversed) {
    this.pattern = pattern;
    this.length = length;
    this.offset = offset;
    this.speed = speed;
    this.reversed = reversed;
  }

}
  

