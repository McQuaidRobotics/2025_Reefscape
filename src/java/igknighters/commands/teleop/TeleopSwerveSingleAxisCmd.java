package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Pose2d;
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
import wayfinder.controllers.Framework.Controller;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Velocity2d;

public class TeleopSwerveSingleAxisCmd extends TeleopSwerveBaseCmd {

  private final Localizer localizer;
  private final Supplier<Rotation2d> headingSupplier;
  private final Supplier<Translation2d> translationSupplier;
  private final Controller<Rotation2d, Double, Rotation2d, Constraints> rotController;
  private final Controller<Translation2d, Velocity2d, Translation2d, Constraints> transController;
  private final ChassisConstraints constraints;
  private final boolean yAxis;

  private Rotation2d lastHeading = Rotation2d.kZero;
  private Translation2d lastTranslation = Translation2d.kZero;

  public TeleopSwerveSingleAxisCmd(
      Swerve swerve,
      DriverController controller,
      Localizer localizer,
      Supplier<Rotation2d> heading,
      Supplier<Translation2d> translation,
      boolean yAxis,
      ChassisConstraints constraints) {
    super(swerve, controller);
    addRequirements(swerve);
    this.localizer = localizer;
    this.headingSupplier = heading;
    this.translationSupplier = translation;
    this.rotController = ControllerFactories.basicRotationalController();
    this.transController = ControllerFactories.shortRangeTranslationController();
    this.constraints = constraints;
    this.yAxis = yAxis;
  }

  private void reset() {
    rotController.reset(
        localizer.pose().getRotation(), swerve.getRobotSpeeds().omega(), headingSupplier.get());
    transController.reset(
        localizer.pose().getTranslation(),
        swerve.getFieldSpeeds().toVelocity2d(),
        translationSupplier.get());
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
    Translation2d translation = this.translationSupplier.get();
    if (!lastHeading.equals(heading) || !lastTranslation.equals(translation)) {
      reset();
      lastHeading = heading;
      lastTranslation = translation;
    }

    Pose2d pose = localizer.pose();
    FieldSpeeds speeds = swerve.getFieldSpeeds();

    double omega =
        rotController.calculate(
            ConstValues.PERIODIC_TIME,
            pose.getRotation(),
            speeds.omega(),
            heading,
            constraints.rotation());

    if (yAxis) {
      double vy =
          transController
              .calculate(
                  ConstValues.PERIODIC_TIME,
                  new Translation2d(translation.getX(), pose.getY()),
                  swerve.getFieldSpeeds().toVelocity2d(),
                  translation,
                  constraints.translation())
              .getVY();
      swerve.drive(Speeds.fromFieldRelative(vt.getX(), vy, omega), constraints);
    } else {
      double vx =
          transController
              .calculate(
                  ConstValues.PERIODIC_TIME,
                  new Translation2d(pose.getX(), translation.getY()),
                  swerve.getFieldSpeeds().toVelocity2d(),
                  translation,
                  constraints.translation())
              .getVX();
      swerve.drive(Speeds.fromFieldRelative(vx, vt.getY(), omega), constraints);
    }
  }

  @Override
  public String getName() {
    return "TeleopSwerveHeadingCmd";
  }
}
