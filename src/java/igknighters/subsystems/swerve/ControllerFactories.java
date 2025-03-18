package igknighters.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.ConstValues.Conv;
import wayfinder.controllers.RotationalController;
import wayfinder.controllers.TranslationController;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wayfinder.controllers.Types.ControllerSequence;
import wpilibExt.Velocity2d;

public class ControllerFactories {
  @SuppressWarnings("unchecked")
  public static Controller<Rotation2d, Double, Rotation2d, Constraints>
      basicRotationalController() {
    return new ControllerSequence<>(
        RotationalController.profiled(5.0, 0, false),
        RotationalController.unprofiled(3.0, 0.1, 1.0 * Conv.DEGREES_TO_RADIANS));
  }

  public static Controller<Rotation2d, Double, Rotation2d, Constraints>
      lowToleranceRotationalController() {
    return RotationalController.unprofiled(4.5, 0.1, 0.0);
  }

  @SuppressWarnings("unchecked")
  public static Controller<Translation2d, Velocity2d, Translation2d, Constraints>
      longRangeTranslationController() {
    return new ControllerSequence<>(
        TranslationController.profiled(5.5, 0, 0, false),
        TranslationController.unprofiled(4.0, 0.0, 0.05, 0.04));
  }

  public static Controller<Translation2d, Velocity2d, Translation2d, Constraints>
      shortRangeTranslationController() {
    return TranslationController.unprofiled(3.0, 0.0, 0.05, 0.04);
  }
}
