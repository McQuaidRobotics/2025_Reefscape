package igknighters.subsystems.superStructure.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

public class ElevatorReal extends Elevator {
  private final TalonFX elevatorFollower;
  private final TalonFX elevatorLeader;

  private final DynamicMotionMagicTorqueCurrentFOC controlReq =
      new DynamicMotionMagicTorqueCurrentFOC(0.0, 0.0, 0.0, 0.0).withUpdateFreqHz(0.0);

  public ElevatorReal() {
    elevatorLeader = new TalonFX(ElevatorConstants.LEADER_ID);
    elevatorFollower = new TalonFX(1);
    elevatorLeader.getConfigurator().apply(elevatorConfiguration());
    elevatorFollower.setControl(new Follower(ElevatorConstants.LEADER_ID, true));
  }

  private TalonFXConfiguration elevatorConfiguration() {

    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = ElevatorConstants.KP;
    cfg.Slot0.kG = ElevatorConstants.KG;
    cfg.Slot0.kD = ElevatorConstants.KD;
    cfg.Slot0.kS = ElevatorConstants.KS;
    cfg.Slot0.kA = ElevatorConstants.KA;

    cfg.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.MAX_HEIGHT + ElevatorConstants.HEIGHT_ABOVE_GROUND;

    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
    cfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
        ElevatorConstants.HEIGHT_ABOVE_GROUND;

    return cfg;
  }

  @Override
  public void gotoPosition(double heightMeters) {
    elevatorLeader.setControl(
        controlReq.withPosition(heightMeters * ElevatorConstants.WHEEL_RADIUS));
  }

  @Override
  public boolean isAtPosition(double heightMeters, double toleranceMeters) {
    return MathUtil.isNear(
        heightMeters, elevatorLeader.getPosition().getValueAsDouble(), toleranceMeters);
  }

  public void setNuetralMode(boolean shouldBeCoast) {
    if (shouldBeCoast) {
      elevatorLeader.setNeutralMode(NeutralModeValue.Coast);
      elevatorFollower.setNeutralMode(NeutralModeValue.Coast);
    } else {
      elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
      elevatorFollower.setNeutralMode(NeutralModeValue.Brake);
    }
  }
}
