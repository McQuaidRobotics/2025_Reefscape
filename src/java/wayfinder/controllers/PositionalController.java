package wayfinder.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import wayfinder.controllers.Types.ChassisConstraints;
import wpilibExt.Speeds.FieldSpeeds;

public class PositionalController {
  private final TranslationController translationController;
  private final RotationalController rotationalController;

  public PositionalController(
      TranslationController translationController, RotationalController rotationalController) {
    this.translationController = translationController;
    this.rotationalController = rotationalController;
  }

  public FieldSpeeds calculate(
      double period,
      Pose2d measurement,
      Pose2d target,
      Transform2d deadband,
      ChassisConstraints constraints) {
    FieldSpeeds translation =
        translationController.calculate(
            period,
            measurement.getTranslation(),
            target.getTranslation(),
            deadband.getTranslation().getNorm(),
            constraints.translation());

    double rotation =
        rotationalController.calculate(
            period,
            measurement.getRotation().getRadians(),
            target.getRotation().getRadians(),
            deadband.getRotation().getRadians(),
            constraints.rotation());

    return new FieldSpeeds(translation.vx(), translation.vy(), rotation);
  }

  public void reset(Pose2d measurement, FieldSpeeds measurementVelo, Pose2d target) {
    translationController.reset(
        measurement.getTranslation(), measurementVelo, target.getTranslation());
    rotationalController.reset(measurement.getRotation().getRadians(), measurementVelo.omega());
  }
}
