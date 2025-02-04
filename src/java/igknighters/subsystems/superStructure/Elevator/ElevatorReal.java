package igknighters.subsystems.superStructure.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.util.can.CANSignalManager;

public class ElevatorReal extends Elevator {
  private final TalonFX elevatorFollower;
  private final TalonFX elevatorLeader;

  private final MotionMagicTorqueCurrentFOC controlReq =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut().withUpdateFreqHz(0.0);

  private final BaseStatusSignal position, velocity, voltage, current;
  private final StatusSignal<ReverseLimitValue> reverseLimit;

  public ElevatorReal() {
    elevatorLeader = new TalonFX(ElevatorConstants.LEADER_ID, ElevatorConstants.CANBUS);
    elevatorFollower = new TalonFX(ElevatorConstants.FOLLOWER_ID, ElevatorConstants.CANBUS);
    elevatorLeader.getConfigurator().apply(elevatorConfiguration());
    elevatorFollower.getConfigurator().apply(elevatorConfiguration());
    elevatorFollower.setControl(new Follower(ElevatorConstants.LEADER_ID, true));

    position = elevatorLeader.getPosition();
    velocity = elevatorLeader.getVelocity();
    voltage = elevatorLeader.getMotorVoltage();
    current = elevatorLeader.getTorqueCurrent();

    reverseLimit = elevatorLeader.getReverseLimit();
    reverseLimit.setUpdateFrequency(1000); // hehe

    CANSignalManager.registerSignals(
        ElevatorConstants.CANBUS, position, velocity, voltage, current);

    CANSignalManager.registerDevices(elevatorLeader, elevatorFollower);
  }

  private TalonFXConfiguration elevatorConfiguration() {

    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = ElevatorConstants.KP * Conv.RADIANS_TO_ROTATIONS;
    cfg.Slot0.kD = ElevatorConstants.KD * Conv.RADIANS_TO_ROTATIONS;
    cfg.Slot0.kS = ElevatorConstants.KS;
    cfg.Slot0.kG = ElevatorConstants.KG;
    cfg.Slot0.kV = ElevatorConstants.KV * Conv.RADIANS_TO_ROTATIONS;

    cfg.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.FORWARD_LIMIT / ElevatorConstants.PULLEY_CIRCUMFERENCE;

    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

    cfg.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY;
    cfg.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;

    cfg.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted =
        ElevatorConstants.INVERT_LEADER
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  @Override
  public void gotoPosition(double targetPosition) {
    super.targetMeters = targetPosition;
    super.controlledLastCycle = true;
    elevatorLeader.setControl(
        controlReq.withPosition(targetPosition / ElevatorConstants.PULLEY_RADIUS));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    if (coast) {
      elevatorLeader.setNeutralMode(NeutralModeValue.Coast);
      elevatorFollower.setNeutralMode(NeutralModeValue.Coast);
    } else {
      elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
      elevatorFollower.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public boolean home() {
    if (!isHomed && super.home()) {
      elevatorLeader.setPosition(ElevatorConstants.REVERSE_LIMIT * Conv.RADIANS_TO_ROTATIONS);
    }
    return isHomed;
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetMeters = Double.NaN;
    super.controlledLastCycle = true;
    elevatorLeader.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetMeters = Double.NaN;
      elevatorLeader.setControl(neutralOut);
    }
    super.controlledLastCycle = false;
    super.meters = position.getValueAsDouble() * ElevatorConstants.PULLEY_CIRCUMFERENCE;
    super.metersPerSecond = velocity.getValueAsDouble() * ElevatorConstants.PULLEY_CIRCUMFERENCE;
    super.volts = voltage.getValueAsDouble();
    super.amps = current.getValueAsDouble();
    super.isLimitTripped = reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }
}
