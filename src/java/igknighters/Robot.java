package igknighters;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.commands.OperatorTarget;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.autos.AutoController;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveTraditionalCmd;
import igknighters.commands.tests.TestManager;
import igknighters.constants.ConstValues;
import igknighters.controllers.DriverController;
import igknighters.controllers.OperatorController;
import igknighters.subsystems.SharedState;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.superStructure.SuperStructure;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.vision.Vision;
import igknighters.util.UnitTestableRobot;
import igknighters.util.can.CANSignalManager;
import igknighters.util.logging.WatchdogSilencer;
import java.io.File;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BiConsumer;
import monologue.Annotations.FlattenedLogged;
import monologue.Annotations.Log;
import monologue.LogSink;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;
import wpilibExt.Tracer;

public class Robot extends UnitTestableRobot<Robot> implements Logged {

  private static final AtomicInteger loopCount = new AtomicInteger(0);

  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  @Log(key = "Localizer")
  public final Localizer localizer = new Localizer();

  public final SimCtx simCtx = new SimCtx(localizer, isSimulation());

  private final DriverController driverController;
  private final OperatorController operatorController;

  @FlattenedLogged public final Subsystems subsystems;

  public final AutoChooser autoChooser = new AutoChooser();
  public final TestManager testManager;

  public Robot() {
    super(ConstValues.PERIODIC_TIME);

    if (isSimulation()) {
      Choreo.setChoreoDir(
          new File(
              Filesystem.getOperatingDirectory(),
              "src" + File.separator + "deploy" + File.separator + "choreo"));
    }

    setupLogging();

    final SharedState sharedState = new SharedState();
    subsystems =
        new Subsystems(
            new Swerve(sharedState, localizer, simCtx),
            new Vision(sharedState, localizer, simCtx),
            new Led(),
            new SuperStructure(sharedState, simCtx),
            new Intake(sharedState, simCtx),
            new Climber(simCtx));

    localizer.reset(
      new Pose2d(
        new Translation2d(12.0, 5.0),
        Rotation2d.kZero
      )
    );

    final var operatorTarget = new OperatorTarget(subsystems, this);
    driverController = new DriverController(0);
    driverController.bind(localizer, subsystems, operatorTarget);
    operatorController = new OperatorController(1);
    operatorController.bind(localizer, subsystems, operatorTarget);
    SubsystemTriggers.setupTriggers(subsystems, localizer, operatorTarget, driverController);

    subsystems.swerve.setDefaultCommand(
        new TeleopSwerveTraditionalCmd(subsystems.swerve, driverController));

    final AutoFactory autoFactory =
        new AutoFactory(
            localizer::pose,
            pose -> {
              localizer.reset(pose);
              subsystems.vision.resetHeading();
              subsystems.swerve.setYaw(pose.getRotation());
            },
            new AutoController(subsystems.swerve, localizer),
            true,
            subsystems.swerve,
            (traj, starting) -> {
              String msg =
                  "[Auto] Trajectory " + traj.name() + " " + (starting ? "Started" : "Finished");
              System.out.println(msg);
              Monologue.log("AutoEvent", msg);
              if (starting) {
                Monologue.log("Auto/Trajectory", traj.samples().toArray(SwerveSample[]::new));
              } else {
                Monologue.log("Auto/Trajectory", new SwerveSample[0]);
              }
            });

    final var routines = new AutoRoutines(subsystems, localizer, autoFactory);
    AutoRoutines.addCmd(autoChooser, "test", routines::test);
    AutoRoutines.addCmd(autoChooser, "testPush", routines::testWithPush);
    autoChooser.addCmd("straight", routines.trajTest("Straight"));
    setupAutoChooser();

    testManager = new TestManager();

    System.gc();
  }

  @Override
  public void robotPeriodic() {
    loopCount.incrementAndGet();
    Tracer.traceFunc("SimCtx", simCtx::update);
    Tracer.traceFunc("CANSignalRefresh", CANSignalManager::refreshSignals);
    Tracer.traceFunc("Localizer", localizer::update);
    Tracer.traceFunc("CommandScheduler", scheduler::run);
    Tracer.traceFunc("Monologue", Monologue::updateAll);
    Tracer.traceFunc(
        "Choosers",
        () -> {
          testManager.update();
        });
  }

  @Override
  public void disabledInit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void autonomousInit() {
    Command autoCmd = autoChooser.selectedCommand();
    String msg = "---- Starting auto command: " + autoCmd.getName() + " ----";
    if (isDebug()) System.out.println(msg);
    Monologue.log("AutoEvent", msg);
    scheduler.schedule(autoCmd);
  }

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void testInit() {
    testManager.getSelectedTestRoutine().schedule();
  }

  @Override
  public void testExit() {
    scheduler.cancelAll();
    System.gc();
  }

  private void setupLogging() {
    WatchdogSilencer.silence(this, "m_watchdog");
    WatchdogSilencer.silence(scheduler, "m_watchdog");

    DriverStation.silenceJoystickConnectionWarning(true);

    // turn off auto logging for signal logger, doesn't get us any info we need
    if (isReal()) {
      SignalLogger.enableAutoLogging(false);
    }

    if (!isUnitTest()) {
      // setup monologue with lazy logging and no datalog prefix
      // robot is the root object
      Monologue.setupMonologue(
          this,
          "/Robot",
          new MonologueConfig().withOptimizeBandwidth(DriverStation::isFMSAttached));
    } else {
      // used for tests and CI, does not actually log anything but asserts the logging is setup
      // mostly correct
      Monologue.setupMonologueDisabled(this, "/Robot", true);
    }

    Monologue.capture("Tracer", NetworkTableInstance.getDefault().getTable("Tracer"));

    // logs build data to the datalog
    final String meta = "/BuildData/";
    Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
    Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
    Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
    Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
    Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
    Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Monologue.log(meta + "GitDirty", "All changes committed");
        break;
      case 1:
        Monologue.log(meta + "GitDirty", "Uncomitted changes");
        break;
      default:
        Monologue.log(meta + "GitDirty", "Unknown");
        break;
    }
    Monologue.log(meta + "Debug", isDebug());
    Monologue.log(meta + "Demo", isDemo());

    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          Monologue.log("Commands/" + command.getName(), active);
        };
    scheduler.onCommandInitialize(
        (Command command) -> {
          logCommandFunction.accept(command, true);
        });
    scheduler.onCommandFinish(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });
    scheduler.onCommandInterrupt(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });

    Monologue.getWpilog().get().flush();
  }

  private void setupAutoChooser() {
    Monologue.publishSendable("/Choosers/AutoChooser", autoChooser, LogSink.NT);
    final StringSubscriber sub =
        NetworkTableInstance.getDefault()
            .getStringTopic("/Choosers/AutoChooser/selected")
            .subscribe(
                "",
                PubSubOption.pollStorage(1),
                PubSubOption.periodic(0.5),
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(false));
    this.addPeriodic(
        () -> {
          var queue = sub.readQueueValues();
          if (queue.length > 0) {
            System.out.println("AutoChooser selected: " + queue[0]);
            autoChooser.select(queue[0]);
          }
        },
        kDefaultPeriod,
        0.01);
  }

  public static int loopCount() {
    return loopCount.get();
  }

  public static boolean isDemo() {
    return ConstValues.DEMO;
  }

  public static boolean isDebug() {
    return ConstValues.DEBUG;
  }
}
