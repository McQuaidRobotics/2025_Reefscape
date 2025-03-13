package igknighters.subsystems.led.driver;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import monologue.Monologue;

public class PWMDriver {

  private final AddressableLED led;
  private final AddressableLEDBuffer previousBuffer;

  // private final AddressableLEDBuffer buffer;

  public PWMDriver(int port) {
    led = new AddressableLED(port);
    led.setLength(73);
    led.start();
    previousBuffer = new AddressableLEDBuffer(73);
  }

  /**
   * will apply a buffer to the LED if its a new one to take up as little resources as possible
   *
   * @param appliedBuffer
   */
  public void applyBuffer(AddressableLEDBuffer appliedBuffer) {
    boolean newBuffer = false;
    if (appliedBuffer == previousBuffer) {
      newBuffer = false;
    } else {
      newBuffer = true;
      led.setData(appliedBuffer);
    }
    Monologue.log("new buffer", newBuffer);
  }

  public void periodic() {}
}
