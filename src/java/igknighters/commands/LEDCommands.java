package igknighters.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import java.util.ArrayList;
import java.util.List;

public class LEDCommands {

  public record LEDSection(int index, int offset, LEDPattern pattern, int length, String name) {}

  public static Command run(
      Led led,
      List<Integer> offsets,
      List<LEDPattern> patterns,
      List<Integer> lengths,
      List<Integer> index,
      List<String> names) {
    final AddressableLEDBuffer slate = new AddressableLEDBuffer(73);
    final LEDPattern eraser = LEDPattern.solid(Color.kBlack);
    return led.startRun(
            () -> {
              eraser.applyTo(slate);
            },
            () -> {
              if (offsets.size() != patterns.size()) {
                DriverStation.reportError(
                    "incorect lengths on offsets and patterns LED COMMANDS ", false);
                return;
              }
              for (int i = 0; i < patterns.size(); i++) {
                if (index.get(i) == 0) {
                  AddressableLEDBufferView controlledZone =
                      slate.createView(
                          MathUtil.clamp(offsets.get(i), 0, 34),
                          MathUtil.clamp(offsets.get(i) + lengths.get(i) - 1, 0, 35));
                  patterns.get(i).applyTo(controlledZone);
                } else {
                  AddressableLEDBufferView controlledZone =
                      slate.createView(
                          MathUtil.clamp(36 + offsets.get(i), 37, slate.getLength() - 1),
                          MathUtil.clamp(
                              36 + offsets.get(i) + lengths.get(i) - 1, 37, slate.getLength() - 1));
                  patterns.get(i).applyTo(controlledZone);
                }
              }
              led.animate(slate);
              LedUtil.logBuffer("fullPattern", led, slate);
            })
        .ignoringDisable(true)
        .withName(
            "split led controlls patterns length = "
                + patterns.size()
                + " offset length = "
                + offsets.size()
                + " led to be controlled: "
                + index);
  }

  public static Command run(Led led, LEDSection... ledSections) {
    List<Integer> offsets = new ArrayList<Integer>();
    List<LEDPattern> patterns = new ArrayList<LEDPattern>();
    List<Integer> lengths = new ArrayList<Integer>();
    List<Integer> indexes = new ArrayList<Integer>();
    List<String> names = new ArrayList<String>();
    for (int i = 0; i < ledSections.length; i++) {
      LEDSection ledSection = ledSections[i];
      offsets.add(ledSection.offset);
      patterns.add(ledSection.pattern);
      lengths.add(ledSection.length);
      indexes.add(ledSection.index);
      names.add(ledSection.name);
    }
    return run(led, offsets, patterns, lengths, indexes, names);
  }

  public static Command run(Led led, LEDPattern pattern) {
    return run(
        led,
        new LEDSection(0, 0, pattern, 36, "full led strip 1"),
        new LEDSection(1, 0, pattern, 37, "full led strip 2"));
  }
}
