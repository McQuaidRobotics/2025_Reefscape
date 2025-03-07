package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements ExclusiveSubsystem {

  public final PWMDriver pwm1;

  public final PWMDriver pwm2;



  public Led() {
    pwm1 = new PWMDriver(9);
    pwm2 = new PWMDriver(0);
    
  }

  public void animate(AddressableLEDBuffer buffer, LEDPattern pattern, int offset, int length, int index) {
    // make our buffer view zones
    if (index == 0) {
      AddressableLEDBufferView controlledZone = buffer.createView(offset, offset + length);
      pattern.applyTo(controlledZone);
      pwm1.applyBuffer(buffer);
    } else {
      AddressableLEDBufferView controlledZone = buffer.createView(offset, offset + length);
      pattern.applyTo(controlledZone);
      pwm2.applyBuffer(buffer);
    }
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    pwm2.periodic();
    Tracer.endTrace();
  }
}
