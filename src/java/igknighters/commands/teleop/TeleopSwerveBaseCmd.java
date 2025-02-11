package igknighters.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableDouble;
import java.util.function.DoubleSupplier;
import wpilibExt.AllianceFlipper;

public class TeleopSwerveBaseCmd extends Command {
  private static boolean shouldOrientForSim() {
    return Robot.isSimulation()
        && TunableValues.getBoolean("OrientTeleopForSim", kSwerve.ORIENT_TELEOP_FOR_SIM_DEFAULT)
            .value();
  }

  /**
   * Orient the chassis speeds for the user, If real robot this means up on translation stick moves
   * away from driver. If simulation and OrientTeleopForSim is true, it will make up on the
   * translation stick move up on the field visualization. If simulation and OrientTeleopForSim is
   * false it will replicate the real robot.
   *
   * @param input The controller input
   * @return The adjusted controller input
   */
  public static Translation2d orientForUser(Translation2d input) {
    if (shouldOrientForSim()) {
      return new Translation2d(input.getX(), -input.getY());
    } else {
      Translation2d chassisSpeedsAdj = input.rotateBy(Rotation2d.fromDegrees(-90));
      return new Translation2d(-chassisSpeedsAdj.getX(), chassisSpeedsAdj.getY());
    }
  }

  protected final Swerve swerve;

  private final DoubleSupplier rawTranslationXSup;
  private final DoubleSupplier rawTranslationYSup;
  private final DoubleSupplier rawRotationXSup;
  private final DoubleSupplier rawRotationYSup;

  private final TunableDouble translationMod;
  private final TunableDouble rotationMod;

  public TeleopSwerveBaseCmd(Swerve swerve, DriverController controller) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.rawTranslationXSup = controller.leftStickX();
    this.rawTranslationYSup = controller.leftStickY();
    this.rawRotationXSup = controller.rightStickX();
    this.rawRotationYSup = controller.rightStickY();

    if (Robot.isDemo()) {
      translationMod = TunableValues.getDouble("DemoSwerveTranslationModifier", 0.8);
      rotationMod = TunableValues.getDouble("DemoSwerveRotationalModifier", 0.8);
    } else {
      translationMod = null;
      rotationMod = null;
    }
  }

  private double invert() {
    if (shouldOrientForSim()) {
      return 1;
    } else if (AllianceFlipper.isRed()) {
      return -1;
    } else {
      return 1;
    }
  }

  protected Translation2d getTranslation() {
    double rawX = -rawTranslationXSup.getAsDouble();
    double rawY = rawTranslationYSup.getAsDouble();
    double angle = Math.atan2(rawY, rawX);
    double rawMagnitude = Math.hypot(rawX, rawY);
    double magnitude = kSwerve.TELEOP_TRANSLATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
    if (Robot.isDemo()) magnitude *= translationMod.value();
    double processedX = magnitude * Math.cos(angle) * invert();
    double processedY = magnitude * Math.sin(angle) * invert();
    return new Translation2d(processedX, processedY);
  }

  protected Translation2d getRotation() {
    double rawX = -rawRotationXSup.getAsDouble();
    double rawY = rawRotationYSup.getAsDouble();
    double angle = Math.atan2(rawY, rawX);
    double rawMagnitude = Math.hypot(rawX, rawY);
    double magnitude = kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
    if (Robot.isDemo()) magnitude *= rotationMod.value();
    double processedX = magnitude * Math.cos(angle);
    double processedY = magnitude * Math.sin(angle);
    return new Translation2d(processedX, processedY);
  }

  protected double getRotationX() {
    double rawX = -rawRotationXSup.getAsDouble();
    double processed = kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawX);
    if (Robot.isDemo()) processed *= rotationMod.value();
    return processed;
  }
}
