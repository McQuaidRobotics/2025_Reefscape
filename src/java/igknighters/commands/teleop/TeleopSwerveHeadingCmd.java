package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.Conv;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.Supplier;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wpilibExt.Speeds;

public class TeleopSwerveHeadingCmd extends TeleopSwerveBaseCmd {

  private final Localizer localizer;
  private final Supplier<Rotation2d> heading;
  private final RotationalController rotController;
  private final ChassisConstraints constraints;

  public TeleopSwerveHeadingCmd(
      Swerve swerve,
      DriverController controller,
      Localizer localizer,
      Supplier<Rotation2d> heading,
      ChassisConstraints constraints) {
    super(swerve, controller);
    addRequirements(swerve);
    this.localizer = localizer;
    this.heading = heading;
    this.rotController = new RotationalController(3.0, 0.1);
    this.constraints = constraints;
  }

  @Override
  public void initialize() {
    rotController.reset(
        localizer.pose().getRotation().getRadians(), 0.0 // TODO
        );
  }

  @Override
  public void execute() {

    Translation2d vt = orientForUser(getTranslation()).times(kSwerve.MAX_DRIVE_VELOCITY);

    double omega =
        rotController.calculate(
            ConstValues.PERIODIC_TIME,
            localizer.pose().getRotation().getRadians(),
            heading.get().getRadians(),
            1.0 * Conv.DEGREES_TO_RADIANS,
            constraints.rotation());

    swerve.drive(Speeds.fromFieldRelative(vt.getX(), vt.getY(), omega), constraints);
  }
}
