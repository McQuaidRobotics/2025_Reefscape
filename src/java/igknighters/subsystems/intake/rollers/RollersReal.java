package igknighters.subsystems.intake.rollers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import igknighters.constants.ConstValues.Conv;

public class RollersReal extends Rollers {
  private final TalonFX intakeLeader = new TalonFX(RollerConstants.INTAKE_LEADER_ID);
  private final TalonFX intakeFollower = new TalonFX(RollerConstants.INTAKE_FOLLOWER_ID);
  private final CANrange distanceSensor = new CANrange(41);
  private final VoltageOut controlReq = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final StatusSignal<ReverseLimitValue> coralSensor;
  private final StatusSignal<Current> currentValue;

  private TalonFXConfiguration intakeConfiguration() {
    var cfg = new TalonFXConfiguration();

    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
    cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    cfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = distanceSensor.getDeviceID();

    return cfg;
  }

  private CANrangeConfiguration intakeSensorConfiguration() {
    var cfg = new CANrangeConfiguration();
    cfg.ProximityParams.ProximityThreshold = Units.inchesToMeters(8.0);
    cfg.ProximityParams.ProximityHysteresis = Units.inchesToMeters(1.0);
    cfg.FovParams.FOVRangeX = 7.0;
    cfg.FovParams.FOVRangeY = 7.0;
    cfg.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
    cfg.ToFParams.UpdateFrequency = 50;

    return cfg;
  }

  public RollersReal() {
    coralSensor = intakeLeader.getReverseLimit();
    this.currentValue = intakeLeader.getTorqueCurrent();
    intakeLeader.getConfigurator().apply(intakeConfiguration());
    distanceSensor.getConfigurator().apply(intakeSensorConfiguration());
    intakeFollower.setControl(new Follower(RollerConstants.INTAKE_LEADER_ID, true));
  }

  @Override
  public void setVoltage(double voltage) {
    intakeLeader.setControl(controlReq.withOutput(voltage));
  }

  @Override
  public boolean hasCoral() {
    return coralSensor.getValue() == ReverseLimitValue.ClosedToGround;
  }

  @Override
  public boolean hasAlgae() {
    final boolean isTripped = hasCoral();
    if (currentValue.refresh().getValueAsDouble() > RollerConstants.ALGAE_TRIP_VALUE && !isTripped) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    amps = currentValue.refresh().getValueAsDouble();
    hasAlgae = hasAlgae();
    hasCoral = hasCoral();
    rads = intakeLeader.getVelocity().getValueAsDouble() * Conv.ROTATIONS_TO_RADIANS;
    log("rpm", intakeLeader.getVelocity().getValueAsDouble()*60.0);
  }
}
