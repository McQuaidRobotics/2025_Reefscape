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
  private final Supplier<Rotation2d> headingSupplier;
  private final RotationalController rotController;
  private final ChassisConstraints constraints;

  private Rotation2d lastHeading = Rotation2d.kZero;

  public TeleopSwerveHeadingCmd(
      Swerve swerve,
      DriverController controller,
      Localizer localizer,
      Supplier<Rotation2d> heading,
      ChassisConstraints constraints) {
    super(swerve, controller);
    addRequirements(swerve);
    this.localizer = localizer;
    this.headingSupplier = heading;
    this.rotController = new RotationalController(3.0, 0.03, false);
    this.constraints = constraints;
  }

  private void reset() {
    rotController.reset(
        localizer.pose().getRotation().getRadians(), swerve.getRobotSpeeds().omega());
  }

  @Override
  public void initialize() {
    reset();
  }

  @Override
  public void execute() {

    Translation2d vt = orientForUser(getTranslation()).times(kSwerve.MAX_DRIVE_VELOCITY);

    Rotation2d heading = this.headingSupplier.get();
    if (!lastHeading.equals(heading)) {
      reset();
      lastHeading = heading;
    }

    double omega =
        rotController.calculate(
            ConstValues.PERIODIC_TIME,
            localizer.pose().getRotation().getRadians(),
            swerve.getRobotSpeeds().omega(),
            heading.getRadians(),
            0.0 * Conv.DEGREES_TO_RADIANS,
            constraints.rotation());

    swerve.drive(Speeds.fromFieldRelative(vt.getX(), vt.getY(), omega), constraints);
  }
}
