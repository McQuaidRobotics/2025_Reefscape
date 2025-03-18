package wayfinder.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Velocity2d;

public class PositionalController
    implements Controller<Pose2d, FieldSpeeds, Pose2d, ChassisConstraints> {
  private final Controller<Translation2d, Velocity2d, Translation2d, Constraints>
      translationController;
  private final Controller<Rotation2d, Double, Rotation2d, Constraints> rotationalController;

  public PositionalController(
      TranslationController translationController, RotationalController rotationalController) {
    this.translationController = translationController;
    this.rotationalController = rotationalController;
  }

  public PositionalController(
      Controller<Translation2d, Velocity2d, Translation2d, Constraints> translationController,
      Controller<Rotation2d, Double, Rotation2d, Constraints> rotationalController) {
    this.translationController = translationController;
    this.rotationalController = rotationalController;
  }

  @Override
  public FieldSpeeds calculate(
      double period,
      Pose2d measurement,
      FieldSpeeds measurementRate,
      Pose2d target,
      ChassisConstraints constraints) {
    var translation =
        translationController.calculate(
            period,
            measurement.getTranslation(),
            measurementRate.toVelocity2d(),
            target.getTranslation(),
            constraints.translation());
    var rotation =
        rotationalController.calculate(
            period,
            measurement.getRotation(),
            measurementRate.omega(),
            target.getRotation(),
            constraints.rotation());
    return new FieldSpeeds(translation.getVX(), translation.getVY(), rotation);
  }

  @Override
  public boolean isDone(Pose2d measurement, Pose2d target) {
    return translationController.isDone(measurement.getTranslation(), target.getTranslation())
        && rotationalController.isDone(measurement.getRotation(), target.getRotation());
  }

  @Override
  public void reset(Pose2d measurement, FieldSpeeds measurementRate, Pose2d target) {
    translationController.reset(
        measurement.getTranslation(), measurementRate.toVelocity2d(), target.getTranslation());
    rotationalController.reset(
        measurement.getRotation(), measurementRate.omega(), target.getRotation());
  }
}
