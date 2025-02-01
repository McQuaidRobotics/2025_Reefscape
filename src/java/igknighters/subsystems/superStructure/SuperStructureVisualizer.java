package igknighters.subsystems.superStructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.SimCtx;
import igknighters.subsystems.superStructure.Elevator.Elevator;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants;
import igknighters.subsystems.superStructure.Wrist.Wrist;

public class SuperStructureVisualizer {
  private final Elevator elevator;
  private final Wrist wrist;
  private final Mechanism2d mechanism;
  
  private final MechanismRoot2d rootCurrent, rootSetpoint;
  
  // private final MechanismLigament2d telescopeCurrent, wristCurrent;
  
  // private final MechanismLigament2d telescopeSetpoint, wristSetpoint;
  
  private final Color8Bit backgroundColor = new Color8Bit(0, 0, 0);
  
  private final Color8Bit debugColor = new Color8Bit(170, 180, 180);
  
  private final Color8Bit boundsColor = new Color8Bit(255, 180, 20);
  
  private final Translation2d pivotOrigin = new Translation2d(0.5, 0.5);
  
  private final double wristOffset = 90;
  
  public SuperStructureVisualizer(SimCtx simCtx, Wrist wrist, Elevator elevator){
    this.elevator = elevator;
    this.wrist = wrist;
    
    mechanism = new Mechanism2d(2.0, 3.0);

    mechanism.setBackgroundColor(backgroundColor);

    rootCurrent = mechanism.getRoot("current", pivotOrigin.getX(), pivotOrigin.getY());
    rootSetpoint = mechanism.getRoot("setpoint", pivotOrigin.getX(), pivotOrigin.getY());

    //telescopeCurrent = rootCurrent.append(new MechanismLigament2d("ElevatorCurrent", ElevatorConstants.MIN_HEIGHT, 90.0));
    // wristCurrent = telescopeCurrent.append(new MechanismLigament2d("Wrist Lower Current", elevator., wristOffset))
  }
}
