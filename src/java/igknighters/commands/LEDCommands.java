package igknighters.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.led.Led;

public class LEDCommands {
  public LEDCommands() {
    final AddressableLEDBuffer buffer = new AddressableLEDBuffer(36);
  }

  /**
   * Continuosly calls led.animate() at the index provided 0 = pwm1 1 = pwm2
   *
   * @param led
   * @param saturation
   * @param value
   * @param index
   * @param velocity
   * @return
   */
  public static Command rainbow(Led led, int saturation, int offset, int length, int value, int index, double velocity) {
    final AddressableLEDBuffer blankSlate = new AddressableLEDBuffer(36);
    final LEDPattern rainbow = LEDPattern.rainbow(saturation, value);
    final LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(velocity), Centimeter.of(1.7));
    ;
    return led.run(
        () -> {
          led.animate(blankSlate, scrollingRainbow, offset, length, index);
        }).withName("rainbow command running on LED strip: " + index);
  }

  record LEDSection(int offset, LEDPattern pattern, int length){}
  
  public static Command runSplit(
      Led led, List<Integer> offsets, List<LEDPattern> patterns, List<Integer> lengths, int index) {
    final AddressableLEDBuffer blankSlate = new AddressableLEDBuffer(36);
    return led.runOnce(
        () -> {
          if (offsets.size() != patterns.size()) {
            DriverStation.reportError(
                "incorect lengths on offsets and patterns LED COMMANDS ", false);
          } else {
            for (int i = 0; i < patterns.size(); i++) {
              led.animate(blankSlate, patterns.get(i), offsets.get(i), lengths.get(i), index);
            }
          }
        }).withName("split led controlls patterns length = " + patterns.size() + " offset length = " + offsets.size() + " led to be controlled: " + index);
  }

  public static Command runSplitWithLEDSection(Led led, int index, LEDSection... ledSections){
    List<Integer> offsets = new ArrayList<Integer>();
    List<LEDPattern> patterns = new ArrayList<LEDPattern>();
    List<Integer> lengths = new ArrayList<Integer>();
    for(int i = 0; i< ledSections.length; i++){
      LEDSection ledSection = ledSections[i];
      offsets.add(ledSection.offset);
      patterns.add(ledSection.pattern);
      lengths.add(ledSection.length);

    }
    return led.runOnce(() -> runSplit(led, offsets, patterns, lengths, index));

  }

  public static Command runFlash(Led led, LEDPattern ledPattern, int speedToFlash){
    return led.runOnce(null);

  }

}
