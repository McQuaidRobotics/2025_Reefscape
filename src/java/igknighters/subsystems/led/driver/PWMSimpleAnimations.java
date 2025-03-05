package igknighters.subsystems.led.driver;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.led.Led;

public class PWMSimpleAnimations extends Led{
  
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  public PWMSimpleAnimations(int port, int length){
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }
  public Command rainbow(int saturation, int value, int speed){
    final LEDPattern rainbowPattern = LEDPattern.rainbow(saturation, value);
    final LEDPattern scrollingRainbowPattern = rainbowPattern.scrollAtAbsoluteSpeed(null, Centimeter.of(0.5));
    return Commands.run(() -> {
      scrollingRainbowPattern.applyTo(buffer);
      led.setData(buffer);
    }).withName("rainbow");

  }
  public Command flashColor(int timePerBlink, Color color){
    final LEDPattern greenPattern = LEDPattern.solid(color);
    greenPattern.blink(Second.of(timePerBlink));
    greenPattern.applyTo(buffer);
    return Commands.run(() -> led.setData(buffer)).withName("flashColor");
  }
  public Command runSplitColors(int[] offsets, LEDPattern[] patterns){
    if (offsets.length != patterns.length){
      throw new IllegalArgumentException("offsets and patterns must be the same length");
    }
    final int prevIndex = 0;
    return Commands.run(() -> {
      for (int i = 0; i< offsets.length; i++){
        final AddressableLEDBufferView view = buffer.createView(prevIndex, offsets[i]);
        patterns[i].applyTo(view);
      }
    }).andThen(() -> led.setData(buffer)).withName("runSplitColors");
  }

}
