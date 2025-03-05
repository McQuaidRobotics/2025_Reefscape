package igknighters.subsystems.led;

import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements SharedSubsystem {
  //private final Driver[] drivers;
  public final PWMDriver pwm1;
  public final PWMDriver pwm2;
  public Led() {
    pwm1 = new PWMDriver(9, 60);
    pwm2 = new PWMDriver(0, 60);
  }


  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    pwm2.periodic();
    Tracer.endTrace();
  }
}
