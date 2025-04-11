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
import igknighters.DeviceManager;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.climber.ClimberConstants;
import igknighters.subsystems.climber.ClimberConstants.PivotConstants;

public class PivotReal extends Pivot {

  private final TalonFX leader =
      new TalonFX(PivotConstants.LEADER_MOTOR_ID, ClimberConstants.CANBUS);
  private final TalonFX follower =
      new TalonFX(PivotConstants.FOLLOWER_MOTOR_ID, ClimberConstants.CANBUS);
  private final CANcoder encoder = new CANcoder(PivotConstants.ENCODER_ID, ClimberConstants.CANBUS);

  private final BaseStatusSignal amps, voltage, encoderPosition;

  private final VoltageOut controlReq = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut().withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final PIDController pidController = new PIDController(20.0, 0.0, 0.0);

  public PivotReal(DeviceManager deviceManager) {
    deviceManager.bringUp(this, "leader", leader, motorConfiguration());
    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deviceManager.bringUp(this, "follower", follower, cfg);
    deviceManager.bringUp(this, "encoder", encoder, encoderConfiguration());

    deviceManager.retryStatusCodeFatal(
        () -> follower.setControl(new Follower(leader.getDeviceID(), true)),
        "set climb follower control request",
        10);

    amps = leader.getStatorCurrent();
    voltage = leader.getMotorVoltage();

    this.radians = encoder.getAbsolutePosition(false).waitForUpdate(2.5).getValue().in(Radians);

    encoder.getAbsolutePosition(false).setUpdateFrequency(125);
    encoder.getPosition(false).setUpdateFrequency(125);
    encoder.getVelocity(false).setUpdateFrequency(125);

    encoderPosition = encoder.getAbsolutePosition(false);
  }

  private final TalonFXConfiguration motorConfiguration() {
    var cfg = new TalonFXConfiguration();

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.FORWARD_LIMIT * Conv.RADIANS_TO_ROTATIONS;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.REVERSE_LIMIT * Conv.RADIANS_TO_ROTATIONS;

    cfg.Voltage.PeakReverseVoltage = -5.5;

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
  public void setPositionRads(double moveToRads) {
    super.targetRads = moveToRads;
    controlledLastCycle = true;
    log("moveToRads", moveToRads);
    leader.setControl(
        controlReq.withOutput(
            log("controllerOutput", pidController.calculate(radians, moveToRads))));
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
