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

  public void animate(AddressableLEDBuffer buffer) {
    prevBuffer = buffer;
    pwm1.applyBuffer(buffer);
  }

  public ArrayList<Color8Bit> getFirst3ColorsOnBoth() {
    ArrayList<Color8Bit> colors = new ArrayList<>();
    for (int i = 0; i < 6; i++) {
      if (i > 2) {
        int index = 37 + i;
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
