package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import wayfinder.controllers.Types.ChassisConstraints;
import wpilibExt.Speeds;

public class TeleopSwerveProfiled extends TeleopSwerveBaseCmd {
  private final ChassisConstraints constraint;

  public TeleopSwerveProfiled(
      Swerve swerve, DriverController controller, ChassisConstraints constraints) {
    super(swerve, controller);
    constraint = constraints;
  }

  @Override
  public void execute() {
    super.execute();
    Translation2d vt = translationStick();

    Speeds fieldSpeeds =
        Speeds.fromFieldRelative(
            vt.getX() * kSwerve.MAX_DRIVE_VELOCITY,
            vt.getY() * kSwerve.MAX_DRIVE_VELOCITY,
            rotationStick().getX() * kSwerve.MAX_ANGULAR_VELOCITY);

    swerve.drive(fieldSpeeds, constraint);
  }
}
