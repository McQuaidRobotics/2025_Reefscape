package igknighters.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import igknighters.Robot;
import igknighters.constants.ConstValues.kLed;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.led.LedAnimations.PartialAnimation;
import igknighters.subsystems.led.driver.CandleDriver;
import igknighters.subsystems.led.driver.Driver;
import igknighters.subsystems.led.driver.PWMDriver;
import igknighters.subsystems.led.driver.SimDriver;
import wpilibExt.Tracer;

public class Led implements SharedSubsystem {
  private final Driver[] driver;

  private int reservedId = 0;
  private boolean reserved = false;

  public Led() {
    if (Robot.isReal()) {
      driver = new Driver[] {new CandleDriver(), new PWMDriver(60, 9)};
    } else {
      driver = new Driver[] {new SimDriver()};
    }
  }
  /**
   * This is the function that is called to animate the LEDs using the animations from the LedAnimations enum (not WPILIB)
   * @param handle
   * @param animation
   */

  public void animate(int handle, LedAnimations animation) {
    if (handle != reservedId && handle >= 0) {
      return;
    }
    for (Driver drivey: driver)
      drivey.animate(
          new PartialAnimation[] {new PartialAnimation(kLed.LED_COUNT, kLed.CANDLE_LEDS, animation)});
  }

  public int reserve() {
    reserved = true;
    return reservedId++;
  }

  public void release(int handle) {
    if (handle == reservedId) {
      reserved = false;
    }
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    for(Driver drivey: driver)
      Tracer.traceFunc("DriverPeriodic", drivey::periodic);

    Tracer.startTrace("DefaultPatternSetter");
    if (!reserved) {
      if (DriverStation.isAutonomousEnabled()) {
        animate(-1, LedAnimations.AUTO);
      } else if (DriverStation.isTeleopEnabled()) {
        animate(-1, LedAnimations.TELEOP);
      } else {
        animate(-1, LedAnimations.DISABLED);
      }
    }
    Tracer.endTrace();

    Tracer.endTrace();
  }
}
