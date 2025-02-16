package igknighters.subsystems.swerve;

import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.commands.swerve.teleop.TeleopSwerveBaseCmd;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.swerve.gyro.Gyro;
import igknighters.subsystems.swerve.gyro.GyroReal;
import igknighters.subsystems.swerve.gyro.GyroSim;
import igknighters.subsystems.swerve.gyro.GyroSimSham;
import igknighters.subsystems.swerve.module.SwerveModule;
import igknighters.subsystems.swerve.module.SwerveModuleReal;
import igknighters.subsystems.swerve.module.SwerveModuleSim;
import igknighters.subsystems.swerve.module.SwerveModuleSimSham;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.subsystems.swerve.odometryThread.SwerveOdometryThread;
import igknighters.util.plumbing.TunableValues;
import java.util.Optional;
import sham.ShamSwerve;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.setpointGenerator.AdvancedSwerveModuleState;
import wayfinder.setpointGenerator.SwerveSetpoint;
import wayfinder.setpointGenerator.SwerveSetpointGenerator;
import wpilibExt.DCMotorExt;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Speeds.RobotSpeeds;
import wpilibExt.Tracer;

/**
 * This is the subsystem for our swerve drivetrain. The Swerve subsystem is composed of 5
 * components, 4 SwerveModules and 1 Gyro. The SwerveModules are the physical wheels and the Gyro is
 * the sensor that measures the robot's rotation. The Swerve subsystem is responsible for
 * controlling the SwerveModules and reading the Gyro.
 *
 * <p>The Swerve subsystem is also responsible for updating the robot's pose and submitting data to
 * the Localizer.
 *
 * <p>{@link
 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html}
 *
 * <p>The coordinate system used in this code is the field coordinate system.
 */
public class Swerve implements ExclusiveSubsystem {
  private final Gyro gyro;
  private final SwerveModule[] swerveMods;
  private final SwerveOdometryThread odometryThread;

  private final SwerveVisualizer visualizer;

  private final SwerveSetpointGenerator setpointGeneratorBeta =
      new SwerveSetpointGenerator(
          new NTEpilogueBackend(NetworkTableInstance.getDefault())
              .getNested("/Robot/Swerve/setpointGenerator"),
          kSwerve.MODULE_CHASSIS_OFFSETS,
          new DCMotorExt(DCMotor.getKrakenX60Foc(1).withReduction(kSwerve.DRIVE_GEAR_RATIO), 1),
          DCMotor.getFalcon500(1).withReduction(kSwerve.STEER_GEAR_RATIO),
          kSwerve.DRIVE_STATOR_CURRENT_LIMIT,
          kSwerve.DRIVE_SUPPLY_CURRENT_LIMIT,
          60.0,
          5.3,
          kSwerve.WHEEL_DIAMETER,
          kSwerve.WHEEL_COF,
          0.0);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(kSwerve.MODULE_CHASSIS_OFFSETS);

  private final Optional<ShamSwerve> sim;

  private Optional<TeleopSwerveBaseCmd> defaultCommand = Optional.empty();
  private SwerveSetpoint setpoint = SwerveSetpoint.zeroed();

  public Swerve(final Localizer localizer, final SimCtx simCtx) {
    final boolean useSham = true;
    if (Robot.isSimulation()) {
      sim = Optional.of((ShamSwerve) simCtx.robot().getDriveTrain());
      final SimSwerveOdometryThread ot =
          new SimSwerveOdometryThread(250, localizer.swerveDataSender());
      if (useSham) {
        swerveMods =
            new SwerveModule[] {
              new SwerveModuleSimSham(0, ot, sim.get()),
              new SwerveModuleSimSham(1, ot, sim.get()),
              new SwerveModuleSimSham(2, ot, sim.get()),
              new SwerveModuleSimSham(3, ot, sim.get()),
            };
        gyro = new GyroSimSham(sim.get().getGyro(), ot);
      } else {
        swerveMods =
            new SwerveModule[] {
              new SwerveModuleSim(0, ot),
              new SwerveModuleSim(1, ot),
              new SwerveModuleSim(2, ot),
              new SwerveModuleSim(3, ot),
            };
        gyro = new GyroSim(this::getRobotSpeeds, ot);
      }
      odometryThread = ot;
    } else {
      sim = Optional.empty();
      final RealSwerveOdometryThread ot =
          new RealSwerveOdometryThread(
              250,
              rots -> (rots / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE,
              localizer.swerveDataSender());
      swerveMods =
          new SwerveModule[] {
            new SwerveModuleReal(0, ot),
            new SwerveModuleReal(1, ot),
            new SwerveModuleReal(2, ot),
            new SwerveModuleReal(3, ot)
          };
      gyro = new GyroReal(ot);
      odometryThread = ot;
    }

    visualizer = new SwerveVisualizer(swerveMods);

    odometryThread.start();
  }

  public void drive(Speeds speeds, ChassisConstraints constraints) {
    RobotSpeeds robotSpeeds = speeds.asRobotRelative(getYaw());
    log("targetSpeed", robotSpeeds);

    if (!Double.isFinite(robotSpeeds.vx())
        || !Double.isFinite(robotSpeeds.vy())
        || !Double.isFinite(robotSpeeds.omega())) {
      DriverStation.reportError("Drivetrain driven at NAN", false);
      drive(FieldSpeeds.kZero, constraints);
      return;
    }

    if (TunableValues.getBoolean("setpointGenerator", true).value()) {
      setpoint =
          setpointGeneratorBeta.generateSetpoint(
              setpoint,
              robotSpeeds.toWpilib(),
              Optional.ofNullable(constraints),
              ConstValues.PERIODIC_TIME);
    } else {
      setpoint =
          setpointGeneratorBeta.generateSimpleSetpoint(
              setpoint, robotSpeeds, ConstValues.PERIODIC_TIME);
    }

    setModuleStates(setpoint.moduleStates());
  }

  public void drive(Speeds speeds) {
    drive(speeds, null);
  }

  /**
   * Offsets the gyro to define the current yaw as the supplied value
   *
   * @param rot A {@link Rotation2d} representing the desired yaw
   */
  public void setYaw(Rotation2d rot) {
    gyro.setYawRads(rot.getRadians());
  }

  /**
   * Gets the current yaw of the robot
   *
   * @return A {@link Rotation2d} representing the current yaw
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(gyro.getYawRads());
  }

  /**
   * Gets the current 3d rotation of the robot
   *
   * @return A {@link Rotation3d} representing the current rotation
   */
  public Rotation3d getRotation() {
    return new Rotation3d(gyro.getRollRads(), gyro.getPitchRads(), gyro.getYawRads());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (SwerveModule module : swerveMods) {
      modulePositions[module.getModuleId()] = module.getCurrentPosition();
    }
    return modulePositions;
  }

  public void setModuleStates(AdvancedSwerveModuleState[] desiredStates) {
    log("regurgitatedSpeed", Speeds.fromRobotRelative(kinematics.toChassisSpeeds(desiredStates)));

    for (SwerveModule module : swerveMods) {
      module.setDesiredState(desiredStates[module.getModuleId()]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : swerveMods) {
      states[module.getModuleId()] = module.getCurrentState();
    }
    return states;
  }

  public RobotSpeeds getRobotSpeeds() {
    return Speeds.fromRobotRelative(kinematics.toChassisSpeeds(getModuleStates()));
  }

  public FieldSpeeds getFieldSpeeds() {
    return getRobotSpeeds().asFieldRelative(getYaw());
  }

  public void setVoltageOut(double voltage, Rotation2d angle) {
    for (SwerveModule module : swerveMods) {
      module.setVoltageOut(voltage, angle);
    }
  }

  @Override
  public void periodic() {
    Tracer.startTrace("SwervePeriodic");

    for (SwerveModule module : swerveMods) {
      Tracer.traceFunc(module.name, module::periodic);
    }

    Tracer.traceFunc("Gyro", gyro::periodic);

    if (DriverStation.isDisabled()) {
      log("targetSpeed", RobotSpeeds.kZero);
    }

    log("measuredSpeed", getRobotSpeeds());

    visualizer.update();

    defaultCommand.ifPresent(
        cmd -> {
          log("Commanded", cmd);
        });

    Tracer.endTrace();
  }

  public void setDefaultCommand(TeleopSwerveBaseCmd defaultCmd) {
    defaultCommand = Optional.of(defaultCmd);
    CommandScheduler.getInstance().setDefaultCommand(this, defaultCmd);
  }
}
