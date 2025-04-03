package igknighters.subsystems.led;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.led.driver.PWMDriver;
import wpilibExt.Tracer;

public class Led implements ExclusiveSubsystem {

  private AddressableLEDBuffer prevBuffer;
  private final PWMDriver pwm1;

  public Led() {
    pwm1 = new PWMDriver(0);
  }

  public void animate(AddressableLEDBuffer buffer) {
    prevBuffer = buffer;
    pwm1.applyBuffer(buffer);
  }

  public List<Color8Bit> getFirst3ColorsOnBoth(){
    ArrayList<Color8Bit> colors = new ArrayList<Color8Bit>();
    return List.of(colors);
  }

  @Override
  public void periodic() {
    Tracer.startTrace("LedPeriodic");
    pwm1.periodic();
    Tracer.endTrace();
  }
}
