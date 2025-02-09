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
import igknighters.constants.ConstValues.Conv;

public class RollersReal extends Rollers {
  private final TalonFX intakeMotor = new TalonFX(RollerConstants.INTAKE_MOTOR_ID);
  private final CANrange distanceSensor = new CANrange(RollerConstants.DISTANCE_SENSOR_ID);

  private final VoltageOut voltageReq = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final Debouncer algaeDebouncer = new Debouncer(0.1, DebounceType.kRising);

  private final StatusSignal<ReverseLimitValue> coralSensor;
  private final BaseStatusSignal current, volts, velocity, temperature;

  private final double kt;

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
    coralSensor = intakeMotor.getReverseLimit();
    current = intakeMotor.getTorqueCurrent();
    volts = intakeMotor.getMotorVoltage();
    velocity = intakeMotor.getVelocity();
    temperature = intakeMotor.getDeviceTemp();
    kt = intakeMotor.getMotorKT().waitForUpdate(1.0).getValueAsDouble();
    intakeMotor.getConfigurator().apply(intakeConfiguration());
    distanceSensor.getConfigurator().apply(intakeSensorConfiguration());
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage > 0.0) {
      algaeDebouncer.calculate(false);
    }
    intakeMotor.setControl(voltageReq.withOutput(voltage));
  }

  @Override
  public void setTorque(double torque) {
    if (torque > 0.0) {
      algaeDebouncer.calculate(false);
    }
    intakeMotor.setControl(currentReq.withOutput(torque / kt));
  }

  @Override
  public boolean hasCoral() {
    return coralSensor.getValue() == ReverseLimitValue.ClosedToGround;
  }

  private boolean probablyHasAlgae() {
    return super.current > RollerConstants.ALGAE_TRIP_VALUE
        && !hasCoral();
  }

  @Override
  public boolean hasAlgae() {
    return algaeDebouncer.calculate(probablyHasAlgae());
  }

  @Override
  public void periodic() {
    super.current = current.getValueAsDouble();
    super.volts = volts.getValueAsDouble();
    super.hasAlgae = hasAlgae();
    super.hasCoral = hasCoral();
    super.radiansPerSecond = velocity.getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    log("rpm", velocity.getValueAsDouble()*60.0);
    log("temp", temperature.getValueAsDouble());
  }
}
