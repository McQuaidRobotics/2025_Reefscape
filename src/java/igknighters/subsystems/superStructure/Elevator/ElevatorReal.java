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
import igknighters.subsystems.superStructure.SuperStructureConstants.kElevator;
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
    leader = new TalonFX(kElevator.LEADER_ID, SuperStructureConstants.CANBUS);
    follower = new TalonFX(kElevator.FOLLOWER_ID, SuperStructureConstants.CANBUS);
    leader.getConfigurator().apply(elevatorConfiguration());
    follower.getConfigurator().apply(elevatorConfiguration());
    follower.setControl(new Follower(kElevator.LEADER_ID, true));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    current = leader.getTorqueCurrent();

    limitSwitch = new DigitalInput(kElevator.LIMIT_SWITCH_ID);

    CANSignalManager.registerSignals(
        SuperStructureConstants.CANBUS, position, velocity, voltage, current);

    CANSignalManager.registerDevices(leader, follower);

    leader.setPosition(kElevator.MIN_HEIGHT / kElevator.PULLEY_CIRCUMFERENCE);
  }

  private TalonFXConfiguration elevatorConfiguration() {

    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = kElevator.kP;
    cfg.Slot0.kD = kElevator.kD;
    cfg.Slot0.kS = kElevator.kS;
    cfg.Slot0.kG = kElevator.kG;
    cfg.Slot0.kV = kElevator.kV / Conv.RADIANS_TO_ROTATIONS;

    cfg.Feedback.SensorToMechanismRatio = kElevator.GEAR_RATIO;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kElevator.MAX_HEIGHT / kElevator.PULLEY_CIRCUMFERENCE;

    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

    cfg.MotionMagic.MotionMagicCruiseVelocity = kElevator.MAX_VELOCITY;
    cfg.MotionMagic.MotionMagicAcceleration = kElevator.MAX_ACCELERATION;

    cfg.CurrentLimits.StatorCurrentLimit = kElevator.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = kElevator.SUPPLY_CURRENT_LIMIT;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted =
        kElevator.INVERT_LEADER
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
      leader.setControl(controlReq.withPosition(targetPosition / kElevator.PULLEY_CIRCUMFERENCE));
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
      leader.setPosition(kElevator.MIN_HEIGHT / kElevator.PULLEY_CIRCUMFERENCE);
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
    super.meters = position.getValueAsDouble() * kElevator.PULLEY_CIRCUMFERENCE;
    super.metersPerSecond = velocity.getValueAsDouble() * kElevator.PULLEY_CIRCUMFERENCE;
    super.volts = voltage.getValueAsDouble();
    super.amps = current.getValueAsDouble();
    super.isLimitTripped = limitSwitch.get();
  }
}
