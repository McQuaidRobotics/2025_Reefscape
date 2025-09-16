package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Localizer;
import igknighters.constants.ConstValues;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.ControllerFactories;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import java.util.function.Supplier;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wpilibExt.Speeds;

public class TeleopSwerveHeadingCmd extends TeleopSwerveBaseCmd {

  private final Localizer localizer;
  private final Supplier<Rotation2d> headingSupplier;
  private final Controller<Rotation2d, Double, Rotation2d, Constraints> rotController;
  private final ChassisConstraints constraints;
  private final boolean fullConstraints;

  private Rotation2d lastHeading = Rotation2d.kZero;

  public TeleopSwerveHeadingCmd(
      Swerve swerve,
      DriverController controller,
      Localizer localizer,
      Supplier<Rotation2d> heading,
      ChassisConstraints constraints,
      boolean fullConstraints) {
    super(swerve, controller);
    addRequirements(swerve);
    this.localizer = localizer;
    this.headingSupplier = heading;
    this.rotController = ControllerFactories.basicRotationalController();
    this.constraints = constraints;
    this.fullConstraints = fullConstraints;
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

    if (fullConstraints) {
      swerve.drive(Speeds.fromFieldRelative(vt.getX(), vt.getY(), omega), constraints);
    } else {
      swerve.drivePreProfiled(Speeds.fromFieldRelative(vt.getX(), vt.getY(), omega));
    }
  }

  @Override
  public String getName() {
    return "TeleopSwerveHeadingCmd";
  }
}
