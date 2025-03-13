package igknighters.subsystems.led;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;
import java.nio.ByteBuffer;
import monologue.Monologue;

public class LedUtil {
  public static final class ColorStruct implements Struct<Color> {
    @Override
    public int getSize() {
      return kSizeDouble * 3;
    }

    @Override
    public Class<Color> getTypeClass() {
      return Color.class;
    }

    @Override
    public String getTypeName() {
      return "Color";
    }

    @Override
    public String getSchema() {
      return "double r; double g; double b;";
    }

    @Override
    public void pack(ByteBuffer bb, Color value) {
      bb.putDouble(value.red);
      bb.putDouble(value.green);
      bb.putDouble(value.blue);
    }

    @Override
    public Color unpack(ByteBuffer bb) {
      return new Color(0.0, 0.0, 0.0);
    }

    public static final ColorStruct INSTANCE = new ColorStruct();
  }

  public static LEDPattern makeRainbow(int saturation, int value) {

    final LEDPattern rainbow = LEDPattern.rainbow(saturation, value);
    final LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.5), Centimeter.of(1.7));
    return scrollingRainbow;
  }

  public static LEDPattern makeFlash(int r, int g, int b, double flashSpeed) {
    final LEDPattern baseColor = LEDPattern.solid(new Color(r, g, b));
    return baseColor.blink(Seconds.of(flashSpeed), Seconds.of(.2));
  }

  public static LEDPattern makeFlash(Color color, double flashSpeed) {
    final LEDPattern baseColor = LEDPattern.solid(color);
    return baseColor.blink(Seconds.of(flashSpeed), Seconds.of(.2));
  }

  public static void logBuffer(AddressableLEDBuffer buffer) {
    final Color[] colors = new Color[2];
    colors[0] = buffer.getLED(0);
    colors[1] = buffer.getLED(39);
    Monologue.log("led0Value", ColorStruct.INSTANCE, colors[0]);
    Monologue.log("led38Val", ColorStruct.INSTANCE, colors[1]);
  }

  public static class NamedLEDPattern implements LEDPattern {
    private final LEDPattern inner;
    private final String name;

    public NamedLEDPattern(String name, LEDPattern inner) {
      this.inner = inner;
      this.name = name;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
      inner.applyTo(reader, writer);
    }

    public String name() {
      return name;
    }
  }
}
