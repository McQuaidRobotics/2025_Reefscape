package igknighters.commands.autos;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import igknighters.Localizer;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.Consumer;
import wpilibExt.Speeds;

public class AutoController implements Consumer<SwerveSample> {
  private final Swerve swerve;
  private final Localizer localizer;
  private final PIDController xController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController rController = new PIDController(5.0, 0.0, 0.0);

  public AutoController(Swerve swerve, Localizer localizer) {
    this.localizer = localizer;
    this.swerve = swerve;
    rController.enableContinuousInput(-Math.PI, Math.PI);
    xController.close();
    yController.close();
    rController.close();
  }

  @Override
  public void accept(SwerveSample referenceState) {
    Pose2d pose = localizer.pose();
    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;

    double xFeedback = xController.calculate(pose.getX(), referenceState.x);
    double yFeedback = yController.calculate(pose.getY(), referenceState.y);
    double rotationFeedback =
        rController.calculate(pose.getRotation().getRadians(), referenceState.heading);

    Speeds out =
        Speeds.fromFieldRelative(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback)
            .asRobotRelative(pose.getRotation());

    swerve.drive(out);
  }
}
