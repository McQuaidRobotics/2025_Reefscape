package igknighters.subsystems.superStructure.Wrist;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.superStructure.SuperStructureConstants;
import igknighters.subsystems.superStructure.SuperStructureConstants.kWrist;
import igknighters.util.can.CANSignalManager;
import java.util.Optional;

public class WristReal extends Wrist {

  private final TalonFX wrist = new TalonFX(kWrist.MOTOR_ID, SuperStructureConstants.CANBUS);
  private final CANcoder encoder = new CANcoder(kWrist.CANCODER_ID, SuperStructureConstants.CANBUS);

  private final BaseStatusSignal position, velocity, amps, voltage;

  private final DynamicMotionMagicVoltage controlReq =
      new DynamicMotionMagicVoltage(
              0.0,
              kWrist.MAX_VELOCITY * Conv.RADIANS_TO_ROTATIONS,
              kWrist.MAX_ACCELERATION * Conv.RADIANS_TO_ROTATIONS,
              0.0)
          .withUpdateFreqHz(0.0);
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

    CANSignalManager.registerSignals(
        SuperStructureConstants.CANBUS, position, velocity, amps, voltage);

    CANSignalManager.registerDevices(wrist, encoder);
  }

  private final TalonFXConfiguration wristConfiguration() {
    var cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted =
        kWrist.INVERT_MOTOR
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    cfg.Slot0.kP = kWrist.KP;
    cfg.Slot0.kD = kWrist.KD;
    cfg.Slot0.kS = kWrist.KS;
    cfg.Slot0.kG = kWrist.KG;
    cfg.Slot0.kV = kWrist.KV * Conv.RADIANS_TO_ROTATIONS;
    cfg.Slot0.kA = kWrist.KA;
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    cfg.Feedback.RotorToSensorRatio = kWrist.GEAR_RATIO;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = kWrist.CANCODER_ID;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kWrist.MIN_ANGLE * Conv.RADIANS_TO_ROTATIONS;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kWrist.MAX_ANGLE * Conv.RADIANS_TO_ROTATIONS;

    cfg.MotionMagic.MotionMagicCruiseVelocity = kWrist.MAX_VELOCITY * Conv.RADIANS_TO_ROTATIONS;
    cfg.MotionMagic.MotionMagicAcceleration = kWrist.MAX_ACCELERATION * Conv.RADIANS_TO_ROTATIONS;

    cfg.CurrentLimits.StatorCurrentLimit = kWrist.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = kWrist.SUPPLY_CURRENT_LIMIT;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted =
        kWrist.INVERT_MOTOR
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  private final CANcoderConfiguration wristCaNcoderConfiguration() {
    var cfg = new CANcoderConfiguration();

    cfg.MagnetSensor.MagnetOffset = kWrist.ANGLE_OFFSET;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cfg.MagnetSensor.SensorDirection =
        kWrist.INVERT_ENCODER
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    return cfg;
  }

  @Override
  public void gotoPosition(double targetPosition, Optional<Constraints> constraints) {
    super.targetRadians = targetPosition;
    super.controlledLastCycle = true;
    final var c = constraints.orElse(DEFAULT_CONSTRAINTS);
    super.maxVelocity = c.maxVelocity;
    super.maxAcceleration = c.maxAcceleration;
    controlReq
        .withVelocity(c.maxVelocity * Conv.RADIANS_TO_ROTATIONS)
        .withAcceleration(c.maxAcceleration * Conv.RADIANS_TO_ROTATIONS);
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
    super.noTarget();
    super.controlledLastCycle = true;
    wrist.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !controlledLastCycle) {
      super.noTarget();
      wrist.setControl(neutralOut);
    }
    super.controlledLastCycle = false;
    super.amps = amps.getValueAsDouble();
    super.volts = voltage.getValueAsDouble();
    super.radians = position.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    super.radiansPerSecond = velocity.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    log("targetDegrees", targetRadians * Conv.RADIANS_TO_DEGREES);
    log("degrees", radians * Conv.RADIANS_TO_DEGREES);
  }
}
