package igknighters.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.intake.IntakeConstants;
import igknighters.subsystems.intake.IntakeConstants.RollerConstants;
import igknighters.util.LerpTable;
import igknighters.util.can.CANSignalManager;

public class RollersReal extends Rollers {
  private static final double INTAKE_WIDTH = 13.5 * Conv.INCHES_TO_METERS;
  private static final double CORAL_WIDTH = 2.25 * Conv.INCHES_TO_METERS;
  private static final LerpTable DISTANCE_LERP =
      new LerpTable(
          new LerpTable.LerpTableEntry(2.5 * Conv.INCHES_TO_METERS, 1.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(4.45 * Conv.INCHES_TO_METERS, 2.5 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(5.25 * Conv.INCHES_TO_METERS, 3.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(5.8 * Conv.INCHES_TO_METERS, 4.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(7.36 * Conv.INCHES_TO_METERS, 5.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(8.937 * Conv.INCHES_TO_METERS, 6.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(9.68 * Conv.INCHES_TO_METERS, 7.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(11.65 * Conv.INCHES_TO_METERS, 8.0 * Conv.INCHES_TO_METERS),
          new LerpTable.LerpTableEntry(13.5 * Conv.INCHES_TO_METERS, 9.25 * Conv.INCHES_TO_METERS));

  private final TalonFX intakeMotor =
      new TalonFX(RollerConstants.INTAKE_MOTOR_ID, IntakeConstants.CANBUS);
  private final CANrange distanceSensor =
      new CANrange(RollerConstants.DISTANCE_SENSOR_ID, IntakeConstants.CANBUS);

  private final VoltageOut voltageReq =
      new VoltageOut(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final StatusSignal<ReverseLimitValue> laserTrippedSignal;
  private final BaseStatusSignal current, voltage, velocity, acceleration, temperature, distance;

  private TalonFXConfiguration intakeConfiguration() {
    var cfg = new TalonFXConfiguration();

    // this can change the latency behavior,
    // we found higher latency was possibly better here but will revisit
    cfg.HardwareLimitSwitch.ReverseLimitEnable = false;
    cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    cfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = distanceSensor.getDeviceID();

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  private CANrangeConfiguration intakeSensorConfiguration() {
    var cfg = new CANrangeConfiguration();
    cfg.ProximityParams.ProximityThreshold = 0.365;
    cfg.ProximityParams.ProximityHysteresis = 0.01;
    cfg.FovParams.FOVRangeX = 7.0;
    cfg.FovParams.FOVRangeY = 7.0;
    // might want to angle this to make detection distance more uniform across the whole intake
    cfg.FovParams.FOVCenterX = 0.0;
    cfg.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return cfg;
  }

  public RollersReal() {
    super(DCMotor.getKrakenX60(1).withReduction(RollerConstants.GEAR_RATIO));
    laserTrippedSignal = intakeMotor.getReverseLimit();
    current = intakeMotor.getTorqueCurrent();
    voltage = intakeMotor.getMotorVoltage();
    velocity = intakeMotor.getVelocity();
    acceleration = intakeMotor.getAcceleration();
    temperature = intakeMotor.getDeviceTemp();
    distance = distanceSensor.getDistance();
    intakeMotor.getConfigurator().apply(intakeConfiguration());
    distanceSensor.getConfigurator().apply(intakeSensorConfiguration());

    CANSignalManager.registerSignals(
        IntakeConstants.CANBUS,
        laserTrippedSignal,
        current,
        voltage,
        velocity,
        acceleration,
        temperature,
        distance);

    CANSignalManager.registerDevices(intakeMotor, distanceSensor);
  }

  @Override
  public void voltageOut(double voltage) {
    super.controlledLastCycle = true;
    intakeMotor.setControl(voltageReq.withOutput(voltage));
  }

  @Override
  public void currentOut(double current) {
    super.controlledLastCycle = true;
    intakeMotor.setControl(currentReq.withOutput(current));
  }

  @Override
  public boolean isStalling() {
    return Math.abs(radiansPerSecond) < 120.0 && Math.abs(amps) > 20.0;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !super.controlledLastCycle) {
      voltageOut(0.0);
    }
    super.controlledLastCycle = false;
    super.amps = current.getValueAsDouble();
    super.volts = voltage.getValueAsDouble();
    super.laserTripped = laserTrippedSignal.getValue() == ReverseLimitValue.ClosedToGround;
    super.gamepieceDistance =
        (DISTANCE_LERP.lerp(distance.getValueAsDouble()) + CORAL_WIDTH) - (INTAKE_WIDTH / 2.0);

    super.radiansPerSecond = velocity.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    log("radiansPerSecondPerSecond", acceleration.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS);
    log("gamepieceDistInches", gamepieceDistance * Conv.METERS_TO_INCHES);
    log("rpm", velocity.getValueAsDouble() * 60.0);
    log("temp", temperature.getValueAsDouble());
    log("isStalling", isStalling());
  }
}
