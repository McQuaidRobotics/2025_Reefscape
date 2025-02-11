package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import wpilibExt.Speeds;

public class TeleopSwerveTraditionalCmd extends TeleopSwerveBaseCmd {

  public TeleopSwerveTraditionalCmd(Swerve swerve, DriverController controller) {
    super(swerve, controller);
  }

  @Override
  public void execute() {
    Translation2d vt = orientForUser(getTranslation());

    Speeds fieldSpeeds =
        Speeds.fromFieldRelative(
            vt.getX() * kSwerve.MAX_DRIVE_VELOCITY,
            vt.getY() * kSwerve.MAX_DRIVE_VELOCITY,
            -getRotationX() * kSwerve.MAX_ANGULAR_VELOCITY * 0.6);

    swerve.drive(fieldSpeeds);
  }
}
