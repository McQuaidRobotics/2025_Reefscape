package igknighters.subsystems.climber.pivot;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.util.can.CANSignalManager;

public class PivotReal extends Pivot {

  private final TalonFX pivotLeader =
      new TalonFX(PivotConstants.LEADER_MOTOR_ID, PivotConstants.CANBUS);
  private final TalonFX pivotFollower =
      new TalonFX(PivotConstants.FOLLOWER_MOTOR_ID, PivotConstants.CANBUS);
  private final CANcoder encoder = new CANcoder(PivotConstants.ENCODER_ID, PivotConstants.CANBUS);

  private final BaseStatusSignal position, velocity, amps, voltage;

  private final PositionDutyCycle controlReq = new PositionDutyCycle(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut().withUpdateFreqHz(0.0);

  public PivotReal() {
    pivotLeader.getConfigurator().apply(motorConfiguration());
    pivotFollower.getConfigurator().apply(motorConfiguration());
    encoder.getConfigurator().apply(encoderConfiguration());

    pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), controlledLastCycle));

    position = pivotLeader.getPosition();
    velocity = pivotLeader.getVelocity();
    amps = pivotLeader.getStatorCurrent();
    voltage = pivotLeader.getMotorVoltage();

    this.radians = encoder.getPosition(false).waitForUpdate(2.5).getValue().in(Radians);

    CANSignalManager.registerSignals(PivotConstants.CANBUS, position, velocity, amps, voltage);

    CANSignalManager.registerDevices(pivotLeader, encoder);
  }

  private final TalonFXConfiguration motorConfiguration() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = PivotConstants.KP;
    cfg.Slot0.kI = PivotConstants.KI;
    cfg.Slot0.kD = PivotConstants.KD;

    cfg.Feedback.RotorToSensorRatio = PivotConstants.GEAR_RATIO;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = PivotConstants.ENCODER_ID;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.FORWARD_LIMIT;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.REVERSE_LIMIT;

    cfg.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.MAX_VELOCITY;
    cfg.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCELERATION;

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
    controlledLastCycle = true;
    super.targetRads = targetRads;
    pivotLeader.setControl(controlReq.withPosition(Conv.RADIANS_TO_ROTATIONS * targetRads));
  }

  @Override
  public void setNeutralMode(boolean coast) {
    if (coast) {
      pivotLeader.setNeutralMode(NeutralModeValue.Coast);
    } else {
      pivotLeader.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.targetRads = Double.NaN;
      pivotLeader.setControl(neutralOut);
    }
    super.controlledLastCycle = false;
    super.amps = amps.getValueAsDouble();
    super.volts = voltage.getValueAsDouble();
    super.radians = position.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
  }
}
