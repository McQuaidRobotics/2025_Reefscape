package igknighters.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.intake.IntakeConstants;
import igknighters.subsystems.intake.IntakeConstants.RollerConstants;
import igknighters.util.can.CANSignalManager;

public class RollersReal extends Rollers {
  private final TalonFX intakeMotor =
      new TalonFX(RollerConstants.INTAKE_MOTOR_ID, IntakeConstants.CANBUS);
  private final CANrange distanceSensor =
      new CANrange(RollerConstants.DISTANCE_SENSOR_ID, IntakeConstants.CANBUS);

  private final VoltageOut voltageReq =
      new VoltageOut(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final Debouncer algaeDebouncer = new Debouncer(0.1, DebounceType.kRising);

  private final StatusSignal<ReverseLimitValue> coralSensor;
  private final BaseStatusSignal current, volts, velocity, temperature;

  private TalonFXConfiguration intakeConfiguration() {
    var cfg = new TalonFXConfiguration();

    // this can change the latency behavior,
    // we found higher latency was possibly better here but will revisit
    cfg.HardwareLimitSwitch.ReverseLimitEnable = false;
    cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    cfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = distanceSensor.getDeviceID();

    return cfg;
  }

  private CANrangeConfiguration intakeSensorConfiguration() {
    var cfg = new CANrangeConfiguration();
    cfg.ProximityParams.ProximityThreshold = 0.26;
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
    coralSensor = intakeMotor.getReverseLimit();
    current = intakeMotor.getTorqueCurrent();
    volts = intakeMotor.getMotorVoltage();
    velocity = intakeMotor.getVelocity();
    temperature = intakeMotor.getDeviceTemp();
    intakeMotor.getConfigurator().apply(intakeConfiguration());
    distanceSensor.getConfigurator().apply(intakeSensorConfiguration());

    CANSignalManager.registerSignals(
        IntakeConstants.CANBUS, coralSensor, current, volts, velocity, temperature);

    CANSignalManager.registerDevices(intakeMotor, distanceSensor);
  }

  @Override
  public void voltageOut(double voltage) {
    super.controlledLastCycle = true;
    if (voltage >= 0.0) {
      algaeDebouncer.calculate(false);
    }
    intakeMotor.setControl(voltageReq.withOutput(voltage));
  }

  @Override
  public void currentOut(double current) {
    super.controlledLastCycle = true;
    if (current >= 0.0) {
      algaeDebouncer.calculate(false);
    }
    intakeMotor.setControl(currentReq.withOutput(current));
  }

  @Override
  public boolean hasCoral() {
    return coralSensor.getValue() == ReverseLimitValue.ClosedToGround;
  }

  private boolean probablyHasAlgae() {
    return super.amps > RollerConstants.ALGAE_TRIP_VALUE && !hasCoral();
  }

  @Override
  public boolean hasAlgae() {
    return algaeDebouncer.calculate(probablyHasAlgae());
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !super.controlledLastCycle) {
      voltageOut(0.0);
    }
    super.controlledLastCycle = false;
    super.amps = current.getValueAsDouble();
    super.volts = volts.getValueAsDouble();
    super.hasAlgae = hasAlgae();
    super.hasCoral = hasCoral();
    super.radiansPerSecond = velocity.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    log("rpm", velocity.getValueAsDouble() * 60.0);
    log("temp", temperature.getValueAsDouble());
  }
}
