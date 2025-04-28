package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements ExclusiveSubsystem {

  public final PWMDriver pwm;

  public Led() {
    pwm = new PWMDriver(0);
  }

  public void animate(AddressableLEDBuffer buffer) {

    pwm.applyBuffer(buffer);
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm.periodic();
    Tracer.endTrace();
  }
}
