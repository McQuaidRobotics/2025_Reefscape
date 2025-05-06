package igknighters.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.ClimberCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.OperatorTarget;
import igknighters.commands.SuperStructureCommands;
import igknighters.commands.SwerveCommands;
import igknighters.commands.teleop.TeleopSwerveHeadingCmd;
// import igknighters.commands.tests.WheelRadiusCharacterization;
// import igknighters.commands.tests.WheelRadiusCharacterization.Direction;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.util.logging.BootupLogger;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableBoolean;
import wpilibExt.AllianceSymmetry;

public class DriverController {
  private final TunableBoolean driveAssist = TunableValues.getBoolean("driverAssist", true);
  private final EventLoop eventLoop = new EventLoop();
  private final Subsystem rumbleResource;
  private double maxObservedMagnitude = 1.0;
  private double rightStickX, rightStickY, leftStickX, leftStickY;

  // Define the bindings for the controller
  @SuppressWarnings("unused")
  public void bind(
      final Localizer localizer, final Subsystems subsystems, final OperatorTarget operatorTarget) {
    final var swerve = subsystems.swerve;
    final var intake = subsystems.intake;
    final var superStructure = subsystems.superStructure;
    final var vision = subsystems.vision;
    final var led = subsystems.led;
    final var climber = subsystems.climber;

    final Trigger shouldAutoAlign =
        new Trigger(
            () -> {
              return driveAssist.value() || DriverStation.isFMSAttached();
            });

    /// FACE BUTTONS
    this.A.or(this.X)
        .whileTrue(IntakeCommands.intakeCoral(intake))
        .whileTrue(
            new TeleopSwerveHeadingCmd(
                    swerve,
                    this,
                    localizer,
                    () -> {
                      final double angle = 54.0;
                      if (localizer.translation().getY() > FieldConstants.FIELD_WIDTH / 2.0) {
                        return AllianceSymmetry.isBlue()
                            ? Rotation2d.fromDegrees(180 - angle)
                            : Rotation2d.fromDegrees(angle);
                      } else {
                        return AllianceSymmetry.isBlue()
                            ? Rotation2d.fromDegrees(-(180 - angle))
                            : Rotation2d.fromDegrees(-angle);
                      }
                    },
                    kSwerve.CONSTRAINTS)
                .onlyIf(shouldAutoAlign)
                .withName("PointToIntake"))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow))
        .onFalse(
            Commands.waitSeconds(0.33)
                .andThen(
                    IntakeCommands.bounce(intake),
                    Commands.waitSeconds(0.1),
                    new ScheduleCommand(IntakeCommands.holdCoral(intake)))
                .withName("BounceThenHoldCoral"));
    this.A.whileTrue(
        SuperStructureCommands.holdAt(superStructure, SuperStructureState.IntakeHpClose));
    this.X.whileTrue(
        SuperStructureCommands.holdAt(superStructure, SuperStructureState.IntakeHpFar));

    this.B.whileTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Processor))
        .whileTrue(
            new TeleopSwerveHeadingCmd(
                    swerve,
                    this,
                    localizer,
                    () -> AllianceSymmetry.isBlue() ? Rotation2d.kCW_Pi_2 : Rotation2d.kCCW_Pi_2,
                    kSwerve.CONSTRAINTS)
                .onlyIf(shouldAutoAlign)
                .withName("PointToProcessor"))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    this.Y.whileTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Net))
        .whileTrue(
            new TeleopSwerveHeadingCmd(
                    swerve,
                    this,
                    localizer,
                    () -> AllianceSymmetry.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg,
                    kSwerve.CONSTRAINTS)
                .onlyIf(shouldAutoAlign)
                .withName("PointToNet"))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    // BUMPER
    this.RB.whileTrue(
        IntakeCommands.expel(
            intake,
            () -> operatorTarget.superStructureState().equals(SuperStructureState.ScoreL1)));

    this.LB.whileTrue(operatorTarget.gotoSuperStructureTargetCmd());

    // CENTER BUTTONS
    this.Back.onTrue(
        Commands.sequence(
                SuperStructureCommands.home(superStructure, true),
                new ScheduleCommand(
                    SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow)))
            .withName("HomeAndHoldStow"));

    this.Start.onTrue(SwerveCommands.orientGyro(swerve, localizer));

    // STICKS
    this.LS.onTrue(Commands.none());

    this.RS.onTrue(operatorTarget.updateTargetCmd(SuperStructureState.ScoreL1, led));

    // // TRIGGERS
    this.LT.and(operatorTarget.hasTarget()).whileTrue(operatorTarget.gotoTargetCmd(this));

    this.RT.onTrue(
        IntakeCommands.bounce(intake)
            .andThen(new ScheduleCommand(IntakeCommands.holdCoral(intake)))
            .withName("IntakeBounce"));

    // DPAD
    this.DPR.onTrue(ClimberCommands.stow(climber));

    this.DPD.onTrue(ClimberCommands.stage(climber));

    this.DPL.whileTrue(Commands.none());

    this.DPU.onTrue(ClimberCommands.climb(climber));

    this.DPR
        .or(this.DPD)
        .or(this.DPL)
        .or(this.DPU)
        .onTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.AntiTilt));

    // COMBOS

    this.LT
        .or(LB)
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow))
        .and(operatorTarget.wantsAlgae().negate())
        .onTrue(IntakeCommands.holdCoral(intake));
    this.LT
        .or(LB)
        .and(operatorTarget.wantsAlgae())
        .whileTrue(IntakeCommands.intakeAlgae(intake))
        .onFalse(IntakeCommands.holdAlgae(intake));
  }

  private final CommandXboxController controller;

  /** Button: 1 */
  protected final Trigger A;

  /** Button: 2 */
  protected final Trigger B;

  /** Button: 3 */
  protected final Trigger X;

  /** Button: 4 */
  protected final Trigger Y;

  /** Left Center; Button: 7 */
  protected final Trigger Back;

  /** Right Center; Button: 8 */
  protected final Trigger Start;

  /** Left Bumper; Button: 5 */
  protected final Trigger LB;

  /** Right Bumper; Button: 6 */
  protected final Trigger RB;

  /** Left Stick; Button: 9 */
  protected final Trigger LS;

  /** Right Stick; Button: 10 */
  protected final Trigger RS;

  /** Left Trigger; Axis: 2 */
  protected final Trigger LT;

  /** Right Trigger; Axis: 3 */
  protected final Trigger RT;

  /** DPad Up; Degrees: 0 */
  protected final Trigger DPU;

  /** DPad Right; Degrees: 90 */
  protected final Trigger DPR;

  /** DPad Down; Degrees: 180 */
  protected final Trigger DPD;

  /** DPad Left; Degrees: 270 */
  protected final Trigger DPL;

  /** for button idx (nice for sim) {@link edu.wpi.first.wpilibj.XboxController.Button} */
  public DriverController(int port) {
    DriverStation.silenceJoystickConnectionWarning(true);
    controller = new CommandXboxController(port);
    rumbleResource =
        new Subsystem() {
          {
            this.setDefaultCommand(
                this.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                    .ignoringDisable(true)
                    .withName("RumbleOff"));
          }
        };
    BootupLogger.bootupLog("Controller " + port + " initialized");
    A = controller.a(eventLoop);
    B = controller.b(eventLoop);
    X = controller.x(eventLoop);
    Y = controller.y(eventLoop);
    LB = controller.leftBumper(eventLoop);
    RB = controller.rightBumper(eventLoop);
    Back = controller.back(eventLoop);
    Start = controller.start(eventLoop);
    LS = controller.leftStick(eventLoop);
    RS = controller.rightStick(eventLoop);
    LT = controller.leftTrigger(0.25, eventLoop);
    RT = controller.rightTrigger(0.25, eventLoop);
    DPR = controller.pov(1, 90, eventLoop);
    DPD = controller.pov(1, 180, eventLoop);
    DPL = controller.pov(1, 270, eventLoop);
    DPU = controller.pov(1, 0, eventLoop);
  }

  public void update() {
    updateJoystickValues(true);
    updateJoystickValues(false);
    eventLoop.poll();
  }

  private void setStickValues(double x, double y, boolean isLeft) {
    if (isLeft) {
      leftStickX = x;
      leftStickY = y;
    } else {
      rightStickX = x;
      rightStickY = y;
    }
  }

  private void updateJoystickValues(boolean isLeft) {
    double x, y, overflow;
    if (isLeft) {
      x = controller.getLeftX();
      y = -controller.getLeftY();
      maxObservedMagnitude = Math.max(maxObservedMagnitude, Math.hypot(x, y));
      overflow = maxObservedMagnitude - 1.0;
    } else {
      x = controller.getRightX();
      y = -controller.getRightY();
      maxObservedMagnitude = Math.max(maxObservedMagnitude, Math.hypot(x, y));
      overflow = maxObservedMagnitude - 1.0;
    }
    if (MathUtil.isNear(0.0, x, 0.01) || MathUtil.isNear(0.0, y, 0.01)) {
      setStickValues(x, y, isLeft);
      return;
    }
    double absX = Math.abs(x);
    double absY = Math.abs(y);
    double diffPercent = 1.0 - (Math.abs(absX - absY) / Math.max(absX, absY));
    overflow *= diffPercent;
    double rawMagnitude = Math.hypot(x, y);
    double magnitude = rawMagnitude / (1.0 + overflow);
    // magnitude = MathUtil.clamp(magnitude, 0, 1.0);
    if (!Double.isFinite(magnitude)) {
      setStickValues(0.0, 0.0, isLeft);
      return;
    }
    double angle = Math.atan2(y, x);
    setStickValues(magnitude * Math.cos(angle), magnitude * Math.sin(angle), isLeft);
  }

  /**
   * Right on the stick is positive (axis 4)
   *
   * @return the value of the right stick x axis
   */
  public double rightStickX() {
    return rightStickX;
  }

  /**
   * Up on the stick is positive (axis 5)
   *
   * @return the value of the right stick y axis
   */
  public double rightStickY() {
    return rightStickY;
  }

  /**
   * Right on the stick is positive (axis 0)
   *
   * @return the value of the left stick x axis
   */
  public double leftStickX() {
    return leftStickX;
  }

  /**
   * Up on the stick is positive (axis 1)
   *
   * @return the value of the left stick y axis
   */
  public double leftStickY() {
    return leftStickY;
  }

  public double rightTrigger() {
    return controller.getRightTriggerAxis();
  }

  public double leftTrigger() {
    return controller.getLeftTriggerAxis();
  }

  /**
   * Will rumble the controller
   *
   * @param type The type of rumble to use
   * @param magnitude The magnitude to rumble at
   * @return The command to rumble the controller
   */
  public Command rumble(RumbleType type, double magnitude) {
    return rumbleResource
        .run(() -> controller.getHID().setRumble(type, magnitude))
        .withName("Rumble");
  }

  /**
   * Will rumble both sides of the controller
   *
   * @param magnitude The magnitude to rumble at
   * @return The command to rumble the controller
   */
  public Command rumble(double magnitude) {
    return rumble(RumbleType.kBothRumble, magnitude);
  }
}
