package igknighters.subsystems.led.driver;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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

public abstract class PWMDriver extends Driver{
  private final AddressableLED led1 = new AddressableLED(LedConstants.LED_PWM_PORT1);
  private final AddressableLED led2 = new AddressableLED(LedConstants.LED_PWM_PORT2);
  private final AddressableLEDBuffer led1Buffer = new AddressableLEDBuffer(LedConstants.LENGTH);
  private final AddressableLEDBuffer led2Buffer = new AddressableLEDBuffer(LedConstants.LENGTH);
  private final LEDPattern patternToRun = LEDPattern.solid(Color.kBlack);
  private PartialAnimation[] animations;

  
  public PWMDriver() {
    led1.setLength(led1Buffer.getLength());
    led2.setLength(led2Buffer.getLength());
    led1.start();
    led2.start();
  }
  @Override
  public void animate(PartialAnimation[] animations) {
    this.animations = animations;

  }
  public void convertAnimationToPWMLibrary(PartialAnimation animation, int length, int stripNumber){
    if(animation.anim().pattern instanceof LedPattern.Rainbow rainbow) {
      final LEDPattern m_rainbow = LEDPattern.rainbow(255, (int)rainbow.brightness());
  
      final Distance kLedSpacing = Meters.of(1 / 120.0);

      final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(rainbow.speed()), kLedSpacing);
      if(stripNumber == 0){
        m_scrollingRainbow.applyTo(led1Buffer);
      }else{
        m_scrollingRainbow.applyTo(led2Buffer);
      }
    } else if(animation.anim().pattern instanceof LedPattern.Solid solid){
      final LEDPattern m_solid = LEDPattern.solid(new Color(solid.r(), solid.b(), solid.b()));
      if (stripNumber == 0){
        m_solid.applyTo(led1Buffer);
      } else{
        m_solid.applyTo(led2Buffer);
      }
    } else if(animation.anim().pattern instanceof LedPattern.Strobe strobe){
      final LEDPattern base = LEDPattern.solid(new Color(strobe.r(), strobe.g(), strobe.b()));
      final LEDPattern m_effect = base.blink(Seconds.of(strobe.speed()));
      if (stripNumber == 0){
        m_effect.applyTo(led1Buffer);
      } else if(stripNumber == 1){
        m_effect.applyTo(led2Buffer);
      }
    } else if(animation.anim().pattern instanceof LedPattern.Flow flow){
      final LEDPattern flowy = LEDPattern.

    }

    }
  

  @Override
  public void periodic() {
    led1.setData(led1Buffer);
    led2.setData(led2Buffer);
  }
  
  
  
}
