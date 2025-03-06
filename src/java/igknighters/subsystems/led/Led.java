package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements SharedSubsystem {

  public final PWMDriver pwm1;

  // public final PWMDriver pwm2;

  public Led() {
    pwm1 = new PWMDriver(9);
    // pwm2 = new PWMDriver(9);
  }

  public void animate(AddressableLEDBuffer buffer, int index) {
    if (index == 0) {
      pwm1.applyBuffer(buffer);
    } else {
      // pwm2.applyBuffer(buffer);
    }
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    // pwm2.periodic();
    Tracer.endTrace();
  }
}
