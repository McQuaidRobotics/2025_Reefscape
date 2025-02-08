package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.util.can.CANSignalManager;

public class WristReal extends Wrist {

  private final TalonFX wrist = new TalonFX(WristConstants.MOTOR_ID, WristConstants.CANBUS);
  private final CANcoder encoder = new CANcoder(WristConstants.CANCODER_ID, WristConstants.CANBUS);

  private final BaseStatusSignal position, velocity, amps, voltage;

  private final MotionMagicTorqueCurrentFOC controlReq =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut =
      new VoltageOut(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut().withUpdateFreqHz(0.0);

  public WristReal() {
    wrist.getConfigurator().apply(wristConfiguration());
    encoder.getConfigurator().apply(wristCaNcoderConfiguration());

    position = wrist.getPosition();
    velocity = wrist.getVelocity();
    amps = wrist.getStatorCurrent();
    voltage = wrist.getMotorVoltage();

    this.radians = encoder.getPosition(false).waitForUpdate(2.5).getValue().in(Radians);

    CANSignalManager.registerSignals(WristConstants.CANBUS, position, velocity, amps, voltage);

    CANSignalManager.registerDevices(wrist, encoder);
  }

  private final TalonFXConfiguration wristConfiguration() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = WristConstants.KP;
    cfg.Slot0.kD = WristConstants.KD;
    cfg.Slot0.kS = WristConstants.KS;
    cfg.Slot0.kG = WristConstants.KG;
    cfg.Slot0.kV = WristConstants.KV;
    cfg.Slot0.kA = WristConstants.KA;

    cfg.Feedback.RotorToSensorRatio = WristConstants.GEAR_RATIO;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = WristConstants.CANCODER_ID;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.FORWARD_LIMIT;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.REVERSE_LIMIT;

    cfg.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MAX_VELOCITY;
    cfg.MotionMagic.MotionMagicAcceleration = WristConstants.MAX_ACCELERATION;

    cfg.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted =
        WristConstants.INVERT_MOTOR
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  private final CANcoderConfiguration wristCaNcoderConfiguration() {
    var cfg = new CANcoderConfiguration();

    cfg.MagnetSensor.MagnetOffset = WristConstants.ANGLE_OFFSET;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cfg.MagnetSensor.SensorDirection =
        WristConstants.INVERT_ENCODER
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    return cfg;
  }

  @Override
  public void goToPosition(double targetPosition) {
    super.targetRadians = targetPosition;
    super.controlledLastCycle = true;
    wrist.setControl(controlReq.withPosition(Conv.RADIANS_TO_ROTATIONS * targetPosition));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    if (coast) {
      wrist.setNeutralMode(NeutralModeValue.Coast);
    } else {
      wrist.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void voltageOut(double voltage) {
    super.targetRadians = Double.NaN;
    super.controlledLastCycle = true;
    wrist.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetRadians = Double.NaN;
      wrist.setControl(neutralOut);
    }
    super.controlledLastCycle = false;
    super.amps = amps.getValueAsDouble();
    super.volts = voltage.getValueAsDouble();
    super.radians = position.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    super.radiansPerSecond = velocity.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
  }
}
