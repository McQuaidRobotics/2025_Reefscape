package igknighters.subsystems.led;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedUtil {
  public static LEDPattern makeRainbow(int saturation, int value){

    final LEDPattern rainbow = LEDPattern.rainbow(saturation, value);
    final LEDPattern scrollingRainbow =
    rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.5), Centimeter.of(1.7));
    return scrollingRainbow;
  
  }

  public static LEDPattern makeFlash(int r, int g, int b, double flashSpeed){
    final LEDPattern baseColor = LEDPattern.solid(new Color(r, g, b));
    baseColor.blink(Seconds.of(flashSpeed));
    return baseColor;
  }
  
}
