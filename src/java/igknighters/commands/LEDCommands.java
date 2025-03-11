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
import monologue.Monologue;

public class LEDCommands {

  public record LEDSection(int index, int offset, LEDPattern pattern, int length, String name) {}

  public static Command runSplit(
      Led led,
      List<Integer> offsets,
      List<LEDPattern> patterns,
      List<Integer> lengths,
      List<Integer> index,
      List<String> names) {
    final AddressableLEDBuffer blankSlate = new AddressableLEDBuffer(72);
    final LEDPattern eraser = LEDPattern.solid(Color.kBlack);
    return led.startRun(
            () -> {
              eraser.applyTo(blankSlate);
            },
            () -> {
              if (offsets.size() != patterns.size()) {
                DriverStation.reportError(
                    "incorect lengths on offsets and patterns LED COMMANDS ", false);
              } else {
                Monologue.log(
                    "both patterns", "PAT 1: " + names.get(0) + " PAT 2: " + names.get(1));
                for (int i = 0; i < patterns.size(); i++) {
                  if (index.get(i) == 0) {
                    Monologue.log(
                        "zone for strip 1",
                        MathUtil.clamp(offsets.get(i), 0, 34)
                            + " endzone: "
                            + MathUtil.clamp(offsets.get(i) + lengths.get(i) - 1, 0, 35));
                    AddressableLEDBufferView controlledZone =
                        blankSlate.createView(
                            MathUtil.clamp(offsets.get(i), 0, 34),
                            MathUtil.clamp(offsets.get(i) + lengths.get(i) - 1, 0, 35));
                    patterns.get(i).applyTo(controlledZone);
                  } else {
                    Monologue.log(
                        "zone for strip 2 ",
                        MathUtil.clamp(36 + offsets.get(i), 37, 69)
                            + " endzone: "
                            + MathUtil.clamp(36 + offsets.get(i) + lengths.get(i) - 1, 37, 71));
                    AddressableLEDBufferView controlledZone =
                        blankSlate.createView(
                            MathUtil.clamp(36 + offsets.get(i), 37, 69),
                            MathUtil.clamp(36 + offsets.get(i) + lengths.get(i) - 1, 37, 72 - 1));
                    patterns.get(i).applyTo(controlledZone);
                  }
                }
                LedUtil.logBuffer(blankSlate);
                led.animate(blankSlate);
              }
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

  public static Command runSplitWithLEDSection(Led led, LEDSection... ledSections) {
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
    return runSplit(led, offsets, patterns, lengths, indexes, names);
  }
}
