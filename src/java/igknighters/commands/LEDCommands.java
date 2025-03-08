package igknighters.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.led.Led;
import java.util.ArrayList;
import java.util.List;
import monologue.Monologue;

public class LEDCommands {
  private final OperatorTarget target;

  public LEDCommands(OperatorTarget target) {
    this.target = target;
  }

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
                for (int i = 0; i < patterns.size(); i++) {
                  Monologue.log("Pattern Name", names.get(i));
                  if (index.get(i) == 0) {
                    AddressableLEDBufferView controlledZone =
                        blankSlate.createView(
                            Math.min(offsets.get(i), 34),
                            Math.min(offsets.get(i) + lengths.get(i) - 1, 36 - 1));
                    patterns.get(i).applyTo(controlledZone);
                  } else {
                    AddressableLEDBufferView controlledZone =
                        blankSlate.createView(
                            Math.min(offsets.get(i), 34),
                            Math.min(offsets.get(i) + lengths.get(i) - 1, 72 - 1));
                    patterns.get(i).applyTo(controlledZone);
                  }
                }
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
