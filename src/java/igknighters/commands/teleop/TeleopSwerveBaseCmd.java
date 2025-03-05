package igknighters.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableDouble;
import java.util.function.DoubleSupplier;
import monologue.ProceduralStructGenerator;
import wpilibExt.AllianceFlipper;

public abstract class TeleopSwerveBaseCmd extends Command {
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

  private double solveJoystickDiagonalDelta(double x, double y) {
    double absX = Math.abs(x);
    double absY = Math.abs(y);
    double diffPercent = 1.0 - (Math.abs(absX - absY) / Math.max(absX, absY));
    double out = Math.max(Math.hypot(x, y) - (0.12 * diffPercent), 0.0);
    if (!Double.isFinite(out)) return 0.0;
    return out;
  }

  protected Translation2d translationStick() {
    double rawX = rawTranslationXSup.getAsDouble();
    double rawY = rawTranslationYSup.getAsDouble();
    double angle = Math.atan2(rawY, rawX);
    double rawMagnitude = solveJoystickDiagonalDelta(rawX, rawY);
    rawMagnitude = MathUtil.clamp(rawMagnitude, -1, 1);
    double magnitude = kSwerve.TELEOP_TRANSLATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
    if (Robot.isDemo()) magnitude *= translationMod.value();
    double processedX = magnitude * Math.cos(angle);
    double processedY = magnitude * Math.sin(angle);
    if (AllianceFlipper.isRed()) {
      return new Translation2d(-processedY, processedX);
    } else {
      return new Translation2d(processedY, -processedX);
    }
  }

  protected Translation2d rotationStick() {
    double rawX = rawRotationXSup.getAsDouble();
    double rawY = rawRotationYSup.getAsDouble();
    double angle = Math.atan2(rawY, rawX);
    double rawMagnitude = Math.hypot(rawX, rawY);
    rawMagnitude = MathUtil.clamp(rawMagnitude, -1, 1);
    double magnitude = kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
    if (Robot.isDemo()) magnitude *= rotationMod.value();
    double processedX = magnitude * Math.cos(angle);
    double processedY = magnitude * Math.sin(angle);
    return new Translation2d(processedX, processedY);
  }

  @Override
  public void execute() {
    swerve.log("teleopCommand", summarize());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.log("teleopCommand", TeleopSwerveCommandSummary.kZero);
  }

  protected record TeleopSwerveCommandSummary(
      double rawTranslationX,
      double translationX,
      double rawTranslationY,
      double translationY,
      double rawRotationX,
      double rotationX,
      double rawRotationY,
      double rotationY)
      implements StructSerializable {
    public static final Struct<TeleopSwerveCommandSummary> struct =
        ProceduralStructGenerator.genRecord(TeleopSwerveCommandSummary.class);

    public static final TeleopSwerveCommandSummary kZero =
        new TeleopSwerveCommandSummary(0, 0, 0, 0, 0, 0, 0, 0);
  }

  protected TeleopSwerveCommandSummary summarize() {
    final Translation2d translation = translationStick();
    final Translation2d rotation = rotationStick();
    return new TeleopSwerveCommandSummary(
        rawTranslationXSup.getAsDouble(),
        translation.getX(),
        rawTranslationYSup.getAsDouble(),
        translation.getY(),
        rawRotationXSup.getAsDouble(),
        rotation.getX(),
        rawRotationYSup.getAsDouble(),
        rotation.getY());
  }
}
