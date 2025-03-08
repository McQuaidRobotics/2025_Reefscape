package igknighters.subsystems.climber.pivot;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.climber.ClimberConstants;
import igknighters.subsystems.climber.ClimberConstants.PivotConstants;
import igknighters.util.can.CANSignalManager;

public class PivotReal extends Pivot {

  private final TalonFX leader =
      new TalonFX(PivotConstants.LEADER_MOTOR_ID, ClimberConstants.CANBUS);
  private final TalonFX follower =
      new TalonFX(PivotConstants.FOLLOWER_MOTOR_ID, ClimberConstants.CANBUS);
  private final CANcoder encoder = new CANcoder(PivotConstants.ENCODER_ID, ClimberConstants.CANBUS);

  private final BaseStatusSignal position, velocity, amps, voltage, encoderPosition;

  private final VoltageOut controlReq = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut().withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final PIDController pidController = new PIDController(60.0, 10.0, 0.0);

  public PivotReal() {
    leader.getConfigurator().apply(motorConfiguration());
    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    follower.getConfigurator().apply(cfg);
    encoder.getConfigurator().apply(encoderConfiguration());

    follower.setControl(new Follower(leader.getDeviceID(), true));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    amps = leader.getStatorCurrent();
    voltage = leader.getMotorVoltage();

    this.radians = encoder.getAbsolutePosition(false).waitForUpdate(2.5).getValue().in(Radians);

    encoder.getAbsolutePosition(false).setUpdateFrequency(125);
    encoder.getPosition(false).setUpdateFrequency(125);
    encoder.getVelocity(false).setUpdateFrequency(125);

    encoderPosition = encoder.getAbsolutePosition(false);

    CANSignalManager.registerSignals(
        ClimberConstants.CANBUS, position, velocity, amps, voltage, encoderPosition);

    CANSignalManager.registerDevices(leader, encoder);
  }

  private final TalonFXConfiguration motorConfiguration() {
    var cfg = new TalonFXConfiguration();

    // cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    // cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     PivotConstants.FORWARD_LIMIT * Conv.RADIANS_TO_ROTATIONS;
    // cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    //     PivotConstants.REVERSE_LIMIT * Conv.RADIANS_TO_ROTATIONS;

    cfg.Voltage.PeakReverseVoltage = 3.0;

    // cfg.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.MAX_VELOCITY;
    // cfg.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCELERATION;

    cfg.CurrentLimits.StatorCurrentLimit = PivotConstants.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted =
        PivotConstants.INVERT_MOTOR
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  private final CANcoderConfiguration encoderConfiguration() {
    var cfg = new CANcoderConfiguration();

    cfg.MagnetSensor.MagnetOffset = PivotConstants.ANGLE_OFFSET;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cfg.MagnetSensor.SensorDirection =
        PivotConstants.INVERT_ENCODER
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    return cfg;
  }

  @Override
  public void setPositionRads(double targetRads) {
    super.targetRads = targetRads;
    controlledLastCycle = true;
    leader.setControl(controlReq.withOutput(pidController.calculate(radians, targetRads)));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    if (coast) {
      leader.setNeutralMode(NeutralModeValue.Coast);
    } else {
      leader.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetRads = Double.NaN;
    controlledLastCycle = true;
    leader.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void periodic() {
    follower.setControl(new Follower(leader.getDeviceID(), true));
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetRads = Double.NaN;
      leader.setControl(neutralOut);
    }
    super.controlledLastCycle = false;
    super.amps = amps.getValueAsDouble();
    super.volts = voltage.getValueAsDouble();
    super.radians = encoderPosition.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
  }
}
