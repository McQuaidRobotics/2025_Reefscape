package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements ExclusiveSubsystem {

  public final PWMDriver pwm1;

  public Led() {
    pwm1 = new PWMDriver(9);
  }

  public void animate(
      AddressableLEDBuffer buffer, LEDPattern pattern, int offset, int length, int index) {
    // make our buffer view zones
    if (index == 0) {
      AddressableLEDBufferView controlledZone = buffer.createView(offset, offset + length - 1);
      pattern.applyTo(controlledZone);
      pwm1.applyBuffer(buffer);
    } else { // offset the pwm buffer by 36 to go on second strip
      AddressableLEDBufferView controlledZone = buffer.createView(offset + 36, offset + length - 1);
      pattern.applyTo(controlledZone);
      pwm1.applyBuffer(buffer);
    }
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    Tracer.endTrace();
  }
}
