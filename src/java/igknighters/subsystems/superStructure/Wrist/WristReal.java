package igknighters.subsystems.superStructure.Wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import igknighters.constants.ConstValues.Conv;

public class WristReal extends Wrist {

  private final TalonFX wrist = new TalonFX(WristConstants.ID);
  private final CANcoder wristCaNcoder = new CANcoder(WristConstants.CANCODER_ID);

  private final DynamicMotionMagicTorqueCurrentFOC controlReq =
      new DynamicMotionMagicTorqueCurrentFOC(0.0, 0.0, 0.0, 0.0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration wristConfiguration() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = WristConstants.KP;
    cfg.Slot0.kD = WristConstants.KD;
    cfg.Slot0.kG = WristConstants.KG;
    cfg.Slot0.kS = WristConstants.KS;
    cfg.Feedback.RotorToSensorRatio = WristConstants.GEAR_RATIO;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = WristConstants.CANCODER_ID;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.MAX_ANGLE;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.MIN_ANGLE;
    return cfg;
  }

  private final CANcoderConfiguration wristCaNcoderConfiguration() {
    var cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.MagnetOffset = WristConstants.ANGLE_OFFSET;
    return cfg;
  }

  public WristReal() {
    wrist.getConfigurator().apply(wristConfiguration());
    wristCaNcoder.getConfigurator().apply(wristCaNcoderConfiguration());
  }

  @Override
  public void goToPosition(double posistionDegrees) {
    wrist.setControl(controlReq.withPosition(Conv.DEGREES_TO_RADIANS * posistionDegrees));
  }

  @Override
  public boolean isAtPosition(double positionDegrees, double toleranceDegrees) {
    return MathUtil.isNear(
        positionDegrees, wristCaNcoder.getPosition().getValueAsDouble(), toleranceDegrees);
  }

  @Override
  public void setNeutralMode(boolean shouldBeCoast) {
    if (shouldBeCoast) {
      wrist.setNeutralMode(NeutralModeValue.Coast);
    } else {
      wrist.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public double positionRadians() {
    return (wristCaNcoder.getPosition().getValueAsDouble() * Math.PI * 2.0);
  }
}
