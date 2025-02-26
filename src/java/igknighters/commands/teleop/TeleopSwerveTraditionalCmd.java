package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import wpilibExt.Speeds;

public class TeleopSwerveTraditionalCmd extends TeleopSwerveBaseCmd {

  public TeleopSwerveTraditionalCmd(Swerve swerve, DriverController controller) {
    super(swerve, controller);
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

    swerve.drive(fieldSpeeds);
  }
}
