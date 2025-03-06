package igknighters.subsystems.led.driver;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import igknighters.subsystems.Component;

public class PWMDriver extends Component {

  private final AddressableLED led;

  private final AddressableLEDBuffer buffer;

  public PWMDriver(int port) {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(40);
    led.setLength(36);
    led.setData(buffer);
    led.start();
  }

  public void applyBuffer(AddressableLEDBuffer appliedBuffer) {
    led.setData(appliedBuffer);
  }

  // public Command rainbow(int saturation, int value, int speed) {
  //   final LEDPattern rainbowPattern = LEDPattern.rainbow(saturation, value);
  //   final LEDPattern scrollingRainbowPattern =
  //       rainbowPattern.scrollAtAbsoluteSpeed(null, Centimeter.of(0.5));
  //   return Commands.run(
  //           () -> {
  //             scrollingRainbowPattern.applyTo(buffer);
  //           })
  //       .withName("rainbow");
  // }

  // public Command flashColor(int timePerBlink, Color color) {
  //   final LEDPattern colorPattern = LEDPattern.solid(color);
  //   colorPattern.blink(Second.of(timePerBlink));
  //   return Commands.run(() -> colorPattern.applyTo(buffer)).withName("flashColor(" + color +
  // ")");
  // }

  // public Command runSplitColors(int[] offsets, LEDPattern[] patterns) {
  //   if (offsets.length != patterns.length) {
  //     throw new IllegalArgumentException("offsets and patterns must be the same length");
  //   }
  //   final int prevIndex = 0;
  //   return Commands.run(
  //           () -> {
  //             for (int i = 0; i < offsets.length; i++) {
  //               final AddressableLEDBufferView view = buffer.createView(prevIndex, offsets[i]);
  //               patterns[i].applyTo(view);
  //             }
  //           })
  //       .withName("runSplitColors");
  // }

  public void periodic() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(buffer);
    led.setData(buffer);
  }
}
