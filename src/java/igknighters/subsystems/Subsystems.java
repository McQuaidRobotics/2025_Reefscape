package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.Intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import monologue.Annotations.Log;
import monologue.Logged;

public class Subsystems implements Logged {
  @Log(key = "Swerve")
  public final Swerve swerve;

  @Log(key = "Vision")
  public final Vision vision;

  @Log(key = "Led")
  public final Led led;

  @Log(key = "SuperStructure")
  public final SuperStructure superStructure;

  @Log(key = "Intake")
  public final Intake intake;

  public Subsystems(
      Swerve swerve, Vision vision, Led led, SuperStructure superStructure, Intake intake) {
    this.swerve = swerve;
    this.led = led;
    this.vision = vision;
    this.superStructure = superStructure;
    this.intake = intake;

    ExclusiveSubsystem[] lockedResources = {swerve, superStructure, intake};
    SharedSubsystem[] locklessResources = {led, vision};

    CommandScheduler.getInstance().registerSubsystem(lockedResources);
    for (SharedSubsystem subsystem : locklessResources) {
      CommandScheduler.getInstance()
          .registerSubsystem(
              new Subsystem() {
                @Override
                public void periodic() {
                  subsystem.periodic();
                }

                @Override
                public String getName() {
                  return subsystem.getName();
                }
              });
    }
  }

  /**
   * @see Subsystem
   */
  public static interface ExclusiveSubsystem extends Subsystem, Logged {}

  /** A subsystem that does not act as a locking resource when using commands */
  public static interface SharedSubsystem extends Logged {
    default void periodic() {}

    default void simulationPeriodic() {}

    default String getName() {
      return this.getClass().getSimpleName();
    }
  }
}
