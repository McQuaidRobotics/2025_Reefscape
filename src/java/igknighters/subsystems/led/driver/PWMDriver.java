package igknighters.subsystems.led.driver;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import igknighters.subsystems.led.LedAnimation;
import igknighters.subsystems.led.LedAnimation.LedPattern;
import igknighters.subsystems.led.LedAnimation.PartialAnimation;
import java.util.Map;

public class PWMDriver extends Driver {
  private final AddressableLED led1;
  private final AddressableLEDBuffer ledBuffer;

  public PWMDriver(int length, int pwm) {
    led1 = new AddressableLED(pwm);
    ledBuffer = new AddressableLEDBuffer(length);
    led1.setLength(ledBuffer.getLength());
    led1.start();
  }
  /**
   * this function applies an animation onto the LED Buffer
   * 
   * @param index how far into the LED strip to start the animation
   * @param length how long the animation should be
   * @param anamation what animation to apply to the LEDs
   * 
   * @return void applys the animation to the LED buffer
   */
  public void animateLEDs(int index, int length, LEDPattern anamation) {
    AddressableLEDBufferView ledView = ledBuffer.createView(index, index + length);
    anamation.applyTo(ledView);


  }
  /**
   * This function calls animate LEDS to apply the animations to the LED buffer using animateLEds and convertAnimationToPWMLibrary
   * 
   * @param animations The desired animations to be played
   */
  @Override
  public void animate(LedAnimation.PartialAnimation[] animations) {
    for(int i = 0; i<animations.length; i++){
      animateLEDs(animations[i].offset(), animations[i].leds(), convertAnimationToPWMLibrary(animations[i]));
    }
    led1.setData(ledBuffer);
  }
  /**
   * Takes a animation from the local LedPattern and converts it into a WPILIB LEDPattern so it can be applied to a LED buffer
   * @param animation
   * @return LEDPattern that is converted from the local LedPattern into the WPILIB LEDPattern
   */
  public LEDPattern convertAnimationToPWMLibrary(PartialAnimation animation) {
    if (animation.anim().pattern instanceof LedPattern.Rainbow rainbowy) {
      return LEDPattern.rainbow(255, (int) rainbowy.brightness());

    } else if (animation.anim().pattern instanceof LedPattern.Solid solid) {
      return LEDPattern.solid(new Color(solid.r(), solid.g(), solid.b()));

    } else if (animation.anim().pattern instanceof LedPattern.Strobe strobe) {
      final LEDPattern base = LEDPattern.solid(new Color(strobe.r(), strobe.g(), strobe.b()));
      final LEDPattern m_effect = base.blink(Seconds.of(strobe.speed()));
      return m_effect;
    } else {
      final LedPattern.Flow flow = (LedPattern.Flow) animation.anim().pattern;
      final Map<Double, Color> map = Map.of(0.0, new Color(flow.r(), flow.g(), flow.b()));
      final LEDPattern base = LEDPattern.solid(Color.kBlack);
      final LEDPattern mask =
          LEDPattern.steps(map)
              .scrollAtAbsoluteSpeed(MetersPerSecond.of(flow.speed()), Centimeter.of(.1));
      base.mask(mask);
      return base;
    }
  }
  /**
   * This function is called by the periodic function in the Led subsystem to update the LEDs to match the Buffer
   */

  @Override
  public void periodic() {
    led1.setData(ledBuffer);
  }
}
