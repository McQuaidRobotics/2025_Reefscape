package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import java.util.ArrayList;
import wpilibExt.Tracer;

public class Led implements ExclusiveSubsystem {

  private AddressableLEDBuffer prevBuffer = new AddressableLEDBuffer(72);
  private final PWMDriver pwm1;

  public Led() {
    pwm1 = new PWMDriver(0);
  }

  /**
   * applies the given buffer to the leds and sets the previous buffer
   *
   * @param buffer
   */
  public void animate(AddressableLEDBuffer buffer) {
    prevBuffer = buffer;
    pwm1.applyBuffer(buffer);
  }

  /**
   * This method returns index 0, 1, 2 on first strip and 37, 38, 39 which is on the other strip
   *
   * @return ArrayList<Color8Bit> first three colors on both street
   */
  public ArrayList<Color8Bit> getFirst3ColorsOnBoth() {
    ArrayList<Color8Bit> colors = new ArrayList<>();
    for (int i = 0; i < 6; i++) {
      if (i > 2) {
        int index = 34 + i;
        colors.add(
            new Color8Bit(
                prevBuffer.getRed(index), prevBuffer.getGreen(index), prevBuffer.getBlue(index)));
      } else {
        colors.add(
            new Color8Bit(prevBuffer.getRed(i), prevBuffer.getGreen(i), prevBuffer.getBlue(i)));
      }
    }
    return colors;
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    Tracer.endTrace();
  }
}
