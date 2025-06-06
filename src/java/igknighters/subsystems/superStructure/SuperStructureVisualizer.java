package igknighters.subsystems.superStructure;

import static igknighters.subsystems.superStructure.SuperStructureConstants.kElevator.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator.Stage;

public class SuperStructureVisualizer {
  private final Mechanism2d mechanism;

  private final MechanismRoot2d rootCurrentStages, rootCurrentCarriage, rootSetpoint;
  private final MechanismLigament2d stage0, stage1, stage2;
  private final MechanismLigament2d carriage, wristStrut1, wristStrut2, intake;
  private final MechanismLigament2d elevatorSetpoint;

  private final Color8Bit backgroundColor = new Color8Bit(0, 0, 0);
  private final Translation2d pivotOrigin = new Translation2d(1.25, Conv.INCHES_TO_METERS * 3.75);
  private final double elevatorWidth = 15.0;
  private final double reefWidth = 7.0;

  public void visualizeReef() {
    final MechanismRoot2d rootReef = mechanism.getRoot("Reef", pivotOrigin.getX() + 1.0, 0.0);

    final MechanismLigament2d reefL1 =
        rootReef.append(new MechanismLigament2d("reefL1", 0.470765190548, 90.0));
    final MechanismLigament2d l1Branch =
        reefL1.append(new MechanismLigament2d("L1Branch", .24614653, 90.0));
    final MechanismLigament2d l1Branch2 =
        l1Branch.append(new MechanismLigament2d("L1Branch2", .447675, 90.0));

    final MechanismLigament2d reefL2 =
        reefL1.append(new MechanismLigament2d("reefL2", .60517315 - .5, 0.0));

    final MechanismLigament2d l2Branch =
        reefL2.append(new MechanismLigament2d("L2Branch", .30783878, 35.0));

    final MechanismLigament2d reefL3 = reefL2.append(new MechanismLigament2d("reefL3", .5, 0.0));

    final MechanismLigament2d l3Branch =
        reefL3.append(new MechanismLigament2d("L3Branch", .3, 35.0));

    final MechanismLigament2d reefL4 = reefL3.append(new MechanismLigament2d("reefL4", 0.5, 0.0));

    final MechanismLigament2d l4Branch =
        reefL4.append(new MechanismLigament2d("L4Branch", .28066193, 45.0));

    final MechanismLigament2d reefL4Arm =
        l4Branch.append(new MechanismLigament2d("reefL1Arm", .20536295, -45.0));

    reefL1.setColor(new Color8Bit(127, 0, 255));
    reefL2.setColor(new Color8Bit(127, 0, 255));
    reefL3.setColor(new Color8Bit(127, 0, 255));
    reefL4.setColor(new Color8Bit(127, 0, 255));

    l1Branch.setColor(new Color8Bit(0, 0, 255));
    l1Branch2.setColor(new Color8Bit(0, 0, 255));
    l2Branch.setColor(new Color8Bit(0, 0, 255));
    l3Branch.setColor(new Color8Bit(0, 0, 255));
    l4Branch.setColor(new Color8Bit(0, 0, 255));

    reefL4Arm.setColor(new Color8Bit(0, 0, 255));

    reefL1.setLineWeight(reefWidth);
    reefL2.setLineWeight(reefWidth);
    reefL3.setLineWeight(reefWidth);
    reefL4.setLineWeight(reefWidth);

    l1Branch.setLineWeight(reefWidth);
    l2Branch.setLineWeight(reefWidth);
    l3Branch.setLineWeight(reefWidth);
    l4Branch.setLineWeight(reefWidth);

    reefL4Arm.setLineWeight(reefWidth);
  }

  public SuperStructureVisualizer() {
    mechanism = new Mechanism2d(2.5, 2.5);
    mechanism.setBackgroundColor(backgroundColor);

    visualizeReef();

    rootSetpoint = mechanism.getRoot("A_setpoint", pivotOrigin.getX(), 0.0);
    rootCurrentStages = mechanism.getRoot("B_stages", pivotOrigin.getX(), pivotOrigin.getY());
    rootCurrentCarriage = mechanism.getRoot("C_carriage", pivotOrigin.getX(), 0.0);

    elevatorSetpoint =
        rootSetpoint.append(new MechanismLigament2d("zElevator Setpoint", 0.0, 90.0));
    elevatorSetpoint.setColor(new Color8Bit(122, 122, 122));
    elevatorSetpoint.setLineWeight(elevatorWidth * 2.0);

    carriage =
        rootCurrentCarriage.append(
            new MechanismLigament2d("Carriage", STAGES[3].tubeLength(), 90.0));
    wristStrut1 =
        carriage.append(
            new MechanismLigament2d(
                "Wrist Strut 1", Math.hypot(3.0, -2.0) * Conv.INCHES_TO_METERS, -135.0));
    wristStrut2 =
        wristStrut1.append(
            new MechanismLigament2d("Wrist Strut 2", wristStrut1.getLength(), -90.0));
    intake =
        wristStrut1.append(new MechanismLigament2d("Intake", 11.0 * Conv.INCHES_TO_METERS, 45.0));

    stage0 =
        rootCurrentStages.append(new MechanismLigament2d("Stage 1", STAGES[0].tubeLength(), 90.0));
    stage1 = stage0.append(new MechanismLigament2d("Stage 2", 0.0, 0.0));
    stage2 = stage1.append(new MechanismLigament2d("Stage 3", 0.0, 0.0));

    carriage.setColor(new Color8Bit(0, 255, 0));
    wristStrut1.setColor(carriage.getColor());
    wristStrut2.setColor(carriage.getColor());
    intake.setColor(new Color8Bit(255, 255, 255));

    stage0.setColor(new Color8Bit(255, 192, 203));
    stage1.setColor(new Color8Bit(255, 0, 0));
    stage2.setColor(new Color8Bit(0, 0, 255));

    carriage.setLineWeight(elevatorWidth);
    stage0.setLineWeight(elevatorWidth);
    stage1.setLineWeight(elevatorWidth);
    stage2.setLineWeight(elevatorWidth);

    // Monologue.publishSendable("/Visualizers/SuperStructure", mechanism, LogSink.OP);
    SmartDashboard.putData("SuperStructure", mechanism);
  }

  /**
   * Updates the visualized setpoint of the superstructure
   *
   * @param elevatorMeters The height of the elevator in meters
   * @param wristRads The angle of the wrist in rads
   */
  public void updateSetpoint(double elevatorMeters, double wristRads) {
    if (Double.isFinite(elevatorMeters)) {
      elevatorSetpoint.setLength(elevatorMeters + (2.0 * Conv.INCHES_TO_METERS));
    } else {
      elevatorSetpoint.setLength(0.0);
    }
  }

  /**
   * Updates the visualized current position of the superstructure
   *
   * @param elevatorMeters The height of the elevator in meters
   * @param wristRads The angle of the wrist in rads
   */
  public void updatePosition(double elevatorMeters, double wristRads) {
    rootCurrentCarriage.setPosition(1.25, elevatorMeters - (6.0 * Conv.INCHES_TO_METERS));

    double firstStageTop = STAGES[0].tubeLength() + pivotOrigin.getY();
    double elevTop = elevatorMeters + (2.0 * Conv.INCHES_TO_METERS) + Stage.TUBE_HEIGHT;
    double distAboveFirst = elevTop - firstStageTop;

    intake.setAngle(45.0 - Math.toDegrees(wristRads));

    if (distAboveFirst < 0.0) {
      stage1.setLength(0.0);
      stage2.setLength(0.0);
    } else if (distAboveFirst > STAGES[2].rangeOfMotion()) {
      stage2.setLength(STAGES[2].rangeOfMotion());
      stage1.setLength(
          Math.min(distAboveFirst - STAGES[2].rangeOfMotion(), STAGES[1].rangeOfMotion()));
    } else {
      stage2.setLength(distAboveFirst);
      stage1.setLength(0.0);
    }
  }
}
