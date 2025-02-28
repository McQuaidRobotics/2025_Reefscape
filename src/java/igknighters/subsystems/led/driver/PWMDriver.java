package igknighters.subsystems.led.driver;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.Component;
import igknighters.subsystems.led.LedAnimations;
import igknighters.subsystems.led.LedConstants;
import igknighters.subsystems.led.LedAnimations.LedPattern;
import igknighters.subsystems.led.LedAnimations.PartialAnimation;
import igknighters.subsystems.led.LedAnimations.LedPattern.Rainbow;
import igknighters.subsystems.led.LedAnimations.LedPattern.Solid;

public class PWMDriver extends Driver{
  private final AddressableLED led1;
  private final AddressableLEDBuffer ledBuffer;
  private PartialAnimation[] animations;
  private LEDPattern pattern;

  
  
  public PWMDriver(int length, int pwm){
    led1 = new AddressableLED(pwm);
    ledBuffer = new AddressableLEDBuffer(length);
    led1.setLength(ledBuffer.getLength());
    led1.start();
  }
  public void animateLEDs(int index, int length, LEDPattern animation){
    AddressableLEDBufferView ledView = ledBuffer.createView(index, index + length);
    animation.applyTo(ledView);

  }
  @Override
  public void animate(LedAnimations.PartialAnimation[] animations) {
    this.animations = animations;
    this.pattern = convertAnimationToPWMLibrary(animations[0]);

  }
  public LEDPattern convertAnimationToPWMLibrary(PartialAnimation animation){
    if(animation.anim().pattern instanceof LedPattern.Rainbow rainbowy) {
      return LEDPattern.rainbow(255, (int)rainbowy.brightness());

    } else if(animation.anim().pattern instanceof LedPattern.Solid solid){
      return LEDPattern.solid(new Color(solid.r(), solid.g(), solid.b()));

    } else if(animation.anim().pattern instanceof LedPattern.Strobe strobe){
      final LEDPattern base = LEDPattern.solid(new Color(strobe.r(), strobe.g(), strobe.b()));
      final LEDPattern m_effect = base.blink(Seconds.of(strobe.speed()));
      return m_effect;
    } else{
      final LedPattern.Flow flow = (LedPattern.Flow) animation.anim().pattern;
      final Map<Double, Color> map = Map.of(0.0, new Color(flow.r(), flow.g(), flow.b()));
      final LEDPattern base = LEDPattern.solid(Color.kBlack);
      final LEDPattern mask = LEDPattern.steps(map).scrollAtAbsoluteSpeed(MetersPerSecond.of(flow.speed()), Centimeter.of(.1));
      base.mask(mask);
      return base;
    }

    }
  

  @Override
  public void periodic() {
    int length = animations.length;
    animateLEDs(0, length, pattern);
  }
  
  
}
