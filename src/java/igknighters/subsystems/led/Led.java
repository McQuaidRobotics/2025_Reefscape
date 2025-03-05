package igknighters.subsystems.led;

import igknighters.Robot;
import igknighters.constants.ConstValues.kLed;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.led.LedAnimation.PartialAnimation;
import igknighters.subsystems.led.driver.Driver;

import igknighters.subsystems.led.driver.SimDriver;
import wpilibExt.Tracer;

public class Led implements SharedSubsystem {
  private final Driver[] drivers;

  public Led() {
    // if (Robot.isReal()) {
    //   // drivers = new Driver[] {
    //   //   new PWMDriver(60, 9),
    //   //   new PWMDriver(60, 0)
    //   };
    // } else {
    //   drivers = new Driver[] {new SimDriver()};
    // }
    drivers = new Driver[] {new SimDriver()};
  }
  /**
   * This is the function that is called to animate the LEDs using the animations from the LedAnimations enum (not WPILIB)
   * @param handle
   * @param animation
   */

  public void animate(int driverIndex, LedAnimation animation) {
    for (Driver driver: drivers) {
      driver.animate(
        new PartialAnimation[] {new PartialAnimation(kLed.LED_COUNT, kLed.CANDLE_LEDS, animation)}
      );
    }
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    for(Driver drivey: drivers) {
      Tracer.traceFunc("DriverPeriodic", drivey::periodic);
    }
    Tracer.endTrace();
  }
}
