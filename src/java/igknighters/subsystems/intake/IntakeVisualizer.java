package igknighters.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {
  Mechanism2d intakeMechanism = new Mechanism2d(40, 40);
  MechanismRoot2d intakeRoot = intakeMechanism.getRoot("rollerPercentOut", 0, 20);
  MechanismLigament2d intakeLigament =
      intakeRoot.append(new MechanismLigament2d("intakeLig", 20, 0));
  Color8Bit fullReverseColor = new Color8Bit(255, 0, 0);
  Color8Bit fullForwardColor = new Color8Bit(0, 255, 0);

  public IntakeVisualizer() {
    intakeLigament.setLineWeight(40);
    intakeMechanism.setBackgroundColor(new Color8Bit(0, 0, 255));
    SmartDashboard.putData("IntakeMechanism", intakeMechanism);
  }

  public void updatePosition(double value) {
    var percent = value / 12.0;

    var color =
        new Color8Bit(
            (int)
                (Math.abs(percent) * (fullForwardColor.red - fullReverseColor.red)
                    + fullReverseColor.red),
            (int)
                (Math.abs(percent) * (fullForwardColor.green - fullReverseColor.green)
                    + fullReverseColor.green),
            (int)
                (Math.abs(percent) * (fullForwardColor.blue - fullReverseColor.blue)
                    + fullReverseColor.blue));
    intakeLigament.setColor(color);
  }
}
