package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements ExclusiveSubsystem {

  public final PWMDriver pwm1;

  public Led() {
    pwm1 = new PWMDriver(0);
  }

  public void animate(AddressableLEDBuffer buffer) {

    pwm1.applyBuffer(buffer);
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    Tracer.endTrace();
  }
}
