package igknighters.subsystems.superStructure.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.MathUtil;
import igknighters.util.can.CANSignalManager;

public class ElevatorReal extends Elevator {
  private final TalonFX elevatorFollower;
  private final TalonFX elevatorLeader;

  private final MotionMagicTorqueCurrentFOC controlReq =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final BaseStatusSignal position, velocity, voltage, current;
  private final StatusSignal<ReverseLimitValue> reverseLimit;

  public ElevatorReal() {
    elevatorLeader = new TalonFX(ElevatorConstants.LEADER_ID, ElevatorConstants.CANBUS);
    elevatorFollower = new TalonFX(ElevatorConstants.FOLLOWER_ID, ElevatorConstants.CANBUS);
    elevatorLeader.getConfigurator().apply(elevatorConfiguration());
    elevatorFollower.setControl(new Follower(ElevatorConstants.LEADER_ID, true));

    position = elevatorLeader.getPosition();
    velocity = elevatorLeader.getVelocity();
    voltage = elevatorLeader.getMotorVoltage();
    current = elevatorLeader.getTorqueCurrent();

    reverseLimit = elevatorLeader.getReverseLimit();
    reverseLimit.setUpdateFrequency(400);

    CANSignalManager.registerSignals(
        ElevatorConstants.CANBUS, position, velocity, voltage, current);
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
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT;

    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

    cfg.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.MAX_VELOCITY / ElevatorConstants.WHEEL_CIRCUMFERENCE;
    cfg.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.WHEEL_CIRCUMFERENCE;

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

  @Override
  public void setNeutralMode(boolean shouldBeCoast) {
    if (shouldBeCoast) {
      elevatorLeader.setNeutralMode(NeutralModeValue.Coast);
      elevatorFollower.setNeutralMode(NeutralModeValue.Coast);
    } else {
      elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
      elevatorFollower.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public boolean home() {
    if (!super.isHomed) {
      elevatorLeader.setControl(voltageOut.withOutput(0.5));
      if (super.isLimitTrip) {
        elevatorLeader.setControl(voltageOut.withOutput(0.0));
        elevatorFollower.setPosition(
            ElevatorConstants.MIN_HEIGHT / ElevatorConstants.WHEEL_CIRCUMFERENCE);
        super.isHomed = true;
      }
      return super.isHomed;
    } else {
      elevatorLeader.setControl(voltageOut.withOutput(0.0));
      return true;
    }
  }

  @Override
  public void periodic() {
    super.meters = position.getValueAsDouble() * ElevatorConstants.WHEEL_CIRCUMFERENCE;
    super.metersPerSecond = velocity.getValueAsDouble() * ElevatorConstants.WHEEL_CIRCUMFERENCE;
    super.volts = voltage.getValueAsDouble();
    super.amps = current.getValueAsDouble();
    super.isLimitTrip = reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }
}
