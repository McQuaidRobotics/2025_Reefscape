package igknighters.subsystems.superStructure.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.SuperStructureConstants;
import igknighters.subsystems.superStructure.SuperStructureConstants.ElevatorConstants;
import igknighters.util.can.CANSignalManager;

public class ElevatorReal extends Elevator {
  private final TalonFX follower;
  private final TalonFX leader;

  private final MotionMagicVoltage controlReq = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut().withUpdateFreqHz(0.0);

  private final BaseStatusSignal position, velocity, voltage, current;
  private final DigitalInput limitSwitch;

  public ElevatorReal() {
    leader = new TalonFX(ElevatorConstants.LEADER_ID, SuperStructureConstants.CANBUS);
    follower = new TalonFX(ElevatorConstants.FOLLOWER_ID, SuperStructureConstants.CANBUS);
    leader.getConfigurator().apply(elevatorConfiguration());
    follower.getConfigurator().apply(elevatorConfiguration());
    follower.setControl(new Follower(ElevatorConstants.LEADER_ID, true));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    current = leader.getTorqueCurrent();

    limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);

    CANSignalManager.registerSignals(
        SuperStructureConstants.CANBUS, position, velocity, voltage, current);

    CANSignalManager.registerDevices(leader, follower);

    leader.setPosition(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  private TalonFXConfiguration elevatorConfiguration() {

    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = ElevatorConstants.KP;
    cfg.Slot0.kD = ElevatorConstants.KD;
    cfg.Slot0.kS = ElevatorConstants.KS;
    cfg.Slot0.kG = ElevatorConstants.KG;
    cfg.Slot0.kV = ElevatorConstants.KV / Conv.RADIANS_TO_ROTATIONS;

    cfg.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.MAX_HEIGHT / ElevatorConstants.PULLEY_CIRCUMFERENCE;

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
    if (isLimitTripped && targetPosition < meters) {
      voltageOut(0.0);
    } else {
      leader.setControl(
          controlReq.withPosition(targetPosition / ElevatorConstants.PULLEY_CIRCUMFERENCE));
    }
  }

  @Override
  public void setNeutralMode(boolean coast) {
    if (coast) {
      leader.setNeutralMode(NeutralModeValue.Coast);
      follower.setNeutralMode(NeutralModeValue.Coast);
    } else {
      leader.setNeutralMode(NeutralModeValue.Brake);
      follower.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public boolean home() {
    if (!isHomed && super.home()) {
      leader.setPosition(ElevatorConstants.MIN_HEIGHT / ElevatorConstants.PULLEY_CIRCUMFERENCE);
    }
    return isHomed;
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetMeters = Double.NaN;
    super.controlledLastCycle = true;
    if (isLimitTripped && voltage < 0.0) {
      voltage = 0.0;
    }
    leader.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetMeters = Double.NaN;
      leader.setControl(neutralOut);
    }
    super.controlledLastCycle = false;
    super.meters = position.getValueAsDouble() * ElevatorConstants.PULLEY_CIRCUMFERENCE;
    super.metersPerSecond = velocity.getValueAsDouble() * ElevatorConstants.PULLEY_CIRCUMFERENCE;
    super.volts = voltage.getValueAsDouble();
    super.amps = current.getValueAsDouble();
    super.isLimitTripped = limitSwitch.get();
  }
}
