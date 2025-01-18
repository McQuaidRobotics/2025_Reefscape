package igknighters.subsystems.superStructure.Wrist;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;

public class WristReal extends Wrist {
  private final TalonFX wrist = new TalonFX(WristConstants.WRIST_ID);
  private final CANcoder wristCaNcoder = new CANcoder(WristConstants.WRIST_CANCODER_ID);
  private final DynamicMotionMagicTorqueCurrentFOC controlReq = new DynamicMotionMagicTorqueCurrentFOC(
      0.0,
      0.0,
      0.0,
      0.0)
      .withUpdateFreqHz(0.0);

  private final TalonFXConfiguration wristConfiguration(){
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = WristConstants.WRIST_KP;
    cfg.Slot0.kG = WristConstants.WRIST_KG;
    cfg.Slot0.kD = WristConstants.WRIST_KD;
    cfg.Slot0.kS = WristConstants.WRIST_KS;
    cfg.Slot0.kA = WristConstants.WRIST_KA;
    cfg.Feedback.SensorToMechanismRatio = WristConstants.WRIST_GEAR_RATIO;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.WRIST_MAX_ANGLE;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.WRIST_MIN_ANGLE;
    return cfg;
  }
  private final CANcoderConfiguration wristCaNcoderConfiguration(){
    var cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.MagnetOffset = WristConstants.WRIST_ANGLE_OFFSET; 
    return cfg;
  }
  public WristReal() {
    wrist.getConfigurator().apply(wristConfiguration());
    wristCaNcoder.getConfigurator().apply(wristCaNcoderConfiguration());
  }
  public void goToPosition(double positionDegrees){
    wrist.setControl(controlReq.withPosition(positionDegrees));
  }

  public boolean isAtPosition(double positionDegrees, double toleranceDegrees){
    return MathUtil.isNear(positionDegrees, wristCaNcoder.getPosition().getValueAsDouble(), toleranceDegrees);
  }
  public void setNeutralMode(boolean shouldBeCoast){
    if(shouldBeCoast){
      wrist.setNeutralMode(NeutralModeValue.Coast);
    } else {
      wrist.setNeutralMode(NeutralModeValue.Brake);
    }
  }
  
}
