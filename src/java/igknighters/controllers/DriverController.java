package igknighters.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.constants.FieldConstants;
import igknighters.constants.Pathing.PathObstacles;
import igknighters.subsystems.Subsystems;

public class DriverController extends ControllerBase {

  public DriverController(int port, Localizer localizer, Subsystems subsystems) {
    super(port, true);

    // disregard null safety for subsystems as it is checked on assignment

    Pose2d face2 = FieldConstants.Reef.CENTER_FACES[4];

    /// FACE BUTTONS
    this.A.onTrue(
        SwerveCommands.moveTo(
            subsystems.swerve,
            localizer,
            new Pose2d(
                face2.getTranslation().plus(new Translation2d(0.33, -0.33)),
                face2.getRotation().plus(Rotation2d.kPi)),
            PathObstacles.FAR_RIGHT_REEF));

    this.B.onTrue(subsystems.swerve.runOnce(() -> {}));

    this.X.onTrue(Commands.none());

    this.Y.onTrue(
        SwerveCommands.moveTo(
            subsystems.swerve,
            localizer,
            new Pose2d(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(-125.0)),
            PathObstacles.Other));

    /// BUMPER
    this.RB.onTrue(Commands.none());

    this.LB.onTrue(Commands.none());

    /// CENTER BUTTONS
    this.Back.onTrue(Commands.none());

    this.Start.onTrue(SwerveCommands.orientGyro(subsystems.swerve, localizer));

    /// STICKS
    this.LS.onTrue(Commands.none());

    this.RS.onTrue(Commands.none());

    /// TRIGGERS
    this.LT.onTrue(Commands.none());

    this.RT.onTrue(Commands.none());

    /// DPAD
    this.DPR.onTrue(Commands.none());

    this.DPD.onTrue(Commands.none());

    this.DPL.onTrue(Commands.none());

    this.DPU.onTrue(Commands.none());
  }
}
