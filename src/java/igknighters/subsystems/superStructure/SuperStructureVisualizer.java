package igknighters.subsystems.superStructure;

import static igknighters.subsystems.superStructure.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.Elevator.ElevatorConstants.Stage;

public class SuperStructureVisualizer {
  private final Mechanism2d mechanism;

  private final MechanismRoot2d rootCurrent, rootSetpoint;
  private final MechanismLigament2d stage1, stage2, stage3Under, carriage, stage3Over;
  private final MechanismLigament2d elevatorSetpoint;

  private final Color8Bit backgroundColor = new Color8Bit(0, 0, 0);
  private final Translation2d pivotOrigin = new Translation2d(1.25, Conv.INCHES_TO_METERS * 3.75);
  private final double elevatorWidth = 15.0;

  public SuperStructureVisualizer() {
    mechanism = new Mechanism2d(2.5, 2.5);
    mechanism.setBackgroundColor(backgroundColor);

    rootSetpoint = mechanism.getRoot("setpoint", pivotOrigin.getX(), 0.0);
    rootCurrent = mechanism.getRoot("current", pivotOrigin.getX(), pivotOrigin.getY());

    elevatorSetpoint = rootSetpoint.append(new MechanismLigament2d("Elevator Setpoint", 0.0, 90.0));
    elevatorSetpoint.setColor(new Color8Bit(122, 122, 122));
    elevatorSetpoint.setLineWeight(elevatorWidth * 2.0);

    stage1 = rootCurrent.append(new MechanismLigament2d("Stage 1", Stage.TUBE_HEIGHT, 90.0));
    stage2 = stage1.append(new MechanismLigament2d("Stage 2", Stage.TUBE_HEIGHT, 0.0));
    stage3Under = stage2.append(new MechanismLigament2d("Stage 3 Under", Stage.TUBE_HEIGHT, 0.0));
    carriage = stage3Under.append(new MechanismLigament2d("Carriage", STAGES[3].tubeLength(), 0.0));
    stage3Over = carriage.append(new MechanismLigament2d("Stage 3 Over", 0.7366, 0.0));

    stage1.setColor(new Color8Bit(255, 192, 203));
    stage2.setColor(new Color8Bit(255, 0, 0));
    stage3Under.setColor(new Color8Bit(0, 0, 255));
    carriage.setColor(new Color8Bit(0, 255, 0));
    stage3Over.setColor(new Color8Bit(0, 0, 255));

    stage1.setLineWeight(elevatorWidth);
    stage2.setLineWeight(elevatorWidth);
    stage3Under.setLineWeight(elevatorWidth);
    carriage.setLineWeight(elevatorWidth);
    stage3Over.setLineWeight(elevatorWidth);

    // Monologue.publishSendable("/Visualizers/SuperStructure", mechanism, LogSink.OP);
    SmartDashboard.putData("SuperStructure", mechanism);
  }

  public void updateSetpoint(double elevatorMeters, double wristRads) {
    elevatorSetpoint.setLength(elevatorMeters);
  }

  public void updatePosition(double elevatorMeters, double wristRads) {
    final double stage3MoveThreshold = Stage.FIRST_STAGE_HEIGHT - Stage.TUBE_HEIGHT;
    final double stage2MoveThreshold = stage3MoveThreshold + STAGES[2].rangeOfMotion();
    final double stage1MoveThreshold = stage2MoveThreshold + STAGES[1].rangeOfMotion();

    double elevExt = elevatorMeters - pivotOrigin.getY();
    double carriageTop = elevExt + (Conv.INCHES_TO_METERS * 2.0);
    double carriageBottom = elevExt - (Conv.INCHES_TO_METERS * 6.0);

    if (carriageTop > stage1MoveThreshold) {
      stage3Over.setLength(Stage.TUBE_HEIGHT);
      stage3Under.setLength(STAGES[2].tubeLength() - STAGES[3].tubeLength() - Stage.OVERLAP);
      stage2.setLength(STAGES[1].rangeOfMotion());
      stage1.setLength(carriageTop - stage1MoveThreshold + Stage.TUBE_HEIGHT);
    } else if (carriageTop > stage2MoveThreshold) {
      stage3Over.setLength(Stage.TUBE_HEIGHT);
      stage3Under.setLength(STAGES[2].tubeLength() - STAGES[3].tubeLength() - Stage.OVERLAP);
      stage2.setLength(carriageTop - stage2MoveThreshold);
      stage1.setLength(Stage.TUBE_HEIGHT);
    } else if (carriageTop > stage3MoveThreshold) {
      stage3Over.setLength(Stage.TUBE_HEIGHT);
      stage3Under.setLength(0.0);
      stage2.setLength(Stage.TUBE_HEIGHT);
      stage1.setLength(Stage.TUBE_HEIGHT);
    } else {
      stage3Over.setLength(0.0);
      stage3Under.setLength(STAGES[2].tubeLength() - STAGES[3].tubeLength() - Stage.OVERLAP);
      stage2.setLength(0.0);
      stage1.setLength(Stage.TUBE_HEIGHT);
    }
  }
}
