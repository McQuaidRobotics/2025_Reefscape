package igknighters.subsystems.led;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microsecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import igknighters.Robot;
import java.nio.ByteBuffer;

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
    return baseColor.blink(Seconds.of(flashSpeed), Seconds.of(flashSpeed));
  }

  public static LEDPattern bounceMaskLayer(LinearVelocity velocity, Distance ledSpacing) {
    return new LEDPattern() {
      final int[] brightnessMask = new int[] {0b11111111, 0b11111100, 0b11110000, 0b11000000};

      double metersPerMicro = velocity.in(Meters.per(Microsecond));
      int microsPerLED = (int) (ledSpacing.in(Meters) / metersPerMicro);

      boolean goingReverse = false;
      int index = 0;
      long lastIncrementTime = RobotController.getTime();

      private int getBrightness(int index) {
        int distanceFromIndex = Math.abs(this.index - index);
        if (distanceFromIndex >= brightnessMask.length) {
          return 0;
        }
        return brightnessMask[distanceFromIndex];
      }

      @Override
      public void applyTo(LEDReader reader, LEDWriter writer) {
        long currentTime = RobotController.getTime();
        while (currentTime - lastIncrementTime > microsPerLED) {
          if (goingReverse) {
            index--;
            if (index == 0) {
              goingReverse = false;
            }
          } else {
            index++;
            if (index == reader.getLength() - 1) {
              goingReverse = true;
            }
          }
          lastIncrementTime += microsPerLED;
        }
        for (int i = 0; i < reader.getLength(); i++) {
          int brightness = getBrightness(i);
          writer.setRGB(i, brightness, brightness, brightness);
        }
      }
    };
  }

  public static LEDPattern makeBounce(Color color, double bounceSpeed) {
    final LEDPattern solid = LEDPattern.solid(color);
    final LEDPattern bounceMask =
        bounceMaskLayer(MetersPerSecond.of(.5 * bounceSpeed), Centimeter.of(1.7));
    return solid.mask(bounceMask);
  }

  public static void logBuffer(String name, Led led, AddressableLEDBuffer buffer) {
    if (Robot.isReal()) {
      return;
    }
    final Color[] colors = new Color[buffer.getLength()];
    for (int i = 0; i < buffer.getLength(); i++) {
      colors[i] = buffer.getLED(i);
    }
    led.log(name, ColorStruct.INSTANCE, colors);
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
