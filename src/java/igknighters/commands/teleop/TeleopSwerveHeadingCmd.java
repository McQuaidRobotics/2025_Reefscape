package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.Conv;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import java.util.function.Supplier;
import wayfinder.controllers.Framework.Controller;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wpilibExt.Speeds;

public class TeleopSwerveHeadingCmd extends TeleopSwerveBaseCmd {

  private final Localizer localizer;
  private final Supplier<Rotation2d> headingSupplier;
  private final Controller<Rotation2d, Double, Rotation2d, Constraints> rotController;
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
    this.rotController = RotationalController.unprofiled(5.5, 0.5, 1.0 * Conv.DEGREES_TO_RADIANS);
    this.constraints = constraints;
  }

  private void reset() {
    rotController.reset(
        localizer.pose().getRotation(), swerve.getRobotSpeeds().omega(), headingSupplier.get());
  }

  @Override
  public void initialize() {
    reset();
  }

  @Override
  public void execute() {
    super.execute();

    Translation2d vt = translationStick().times(kSwerve.MAX_DRIVE_VELOCITY);

    Rotation2d heading = this.headingSupplier.get();
    if (!lastHeading.equals(heading)) {
      reset();
      lastHeading = heading;
    }

    double omega =
        rotController.calculate(
            ConstValues.PERIODIC_TIME,
            localizer.pose().getRotation(),
            swerve.getRobotSpeeds().omega(),
            heading,
            constraints.rotation());

    swerve.drivePreProfiled(Speeds.fromFieldRelative(vt.getX(), vt.getY(), omega));
  }

  @Override
  public String getName() {
    return "TeleopSwerveHeadingCmd";
  }
}
