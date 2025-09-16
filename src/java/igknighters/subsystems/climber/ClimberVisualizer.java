package igknighters.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimberVisualizer {
  private final Mechanism2d climber;
  private final MechanismRoot2d climberRoot;
  private final MechanismLigament2d longStick, hook;

  public ClimberVisualizer() {
    climber = new Mechanism2d(50, 50);
    climberRoot = climber.getRoot("climberMech", 25, 25);
    longStick =
        climberRoot.append(
            new MechanismLigament2d("longStick", 20, 0, 5, new Color8Bit(0, 255, 0)));
    hook = longStick.append(new MechanismLigament2d("hook", 7, 90, 5, new Color8Bit(255, 0, 0)));
    SmartDashboard.putData("Climber", climber);
  }

  public void updatePosition(double angleInDegrees) {
    longStick.setAngle(angleInDegrees);
  }
}
