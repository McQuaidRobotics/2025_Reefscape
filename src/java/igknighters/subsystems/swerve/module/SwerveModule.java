package igknighters.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import igknighters.subsystems.Component;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants;
import monologue.Annotations.Log;
import wayfinder.setpointGenerator.AdvancedSwerveModuleState;

public abstract class SwerveModule extends Component {
  @Log public double driveVeloMPS = 0.0;
  @Log public double targetDriveVeloMPS = 0.0;
  @Log public double drivePositionMeters = 0.0;
  @Log public double driveVolts = 0.0;
  @Log public double driveAmps = 0.0;
  @Log public double steerVeloRadPS = 0.0;
  @Log public double steerAbsoluteRads = 0.0;
  @Log public double targetSteerAbsoluteRads = 0.0;
  @Log public double steerVolts = 0.0;
  @Log public double steerAmps = 0.0;
  @Log protected boolean controlledLastCycle;

  public final String name;

  protected SwerveModule(String name) {
    this.name = name;
  }

  /**
   * @param desiredState The state that the module should assume, angle and velocity.
   * @param isOpenLoop Whether the module speed assumed should be reached via open or closed loop
   *     control.
   */
  public abstract void setDesiredState(AdvancedSwerveModuleState desiredState);

  /**
   * @return The velocity/angle of the module.
   */
  public abstract SwerveModuleState getCurrentState();

  public abstract SwerveModulePosition getCurrentPosition();

  public abstract int getModuleId();

  public abstract void setVoltageOut(double volts, Rotation2d angle);

  protected double getOffset(int moduleId) {
    return ModuleConstants.kSteerEncoder.ENCODER_OFFSETS_ROTATIONS[moduleId];
  }
}
