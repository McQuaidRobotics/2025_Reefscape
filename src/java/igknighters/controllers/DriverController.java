package igknighters.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.ClimberCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.OperatorTarget;
import igknighters.commands.SuperStructureCommands;
import igknighters.commands.SwerveCommands;
import igknighters.commands.teleop.TeleopSwerveHeadingCmd;
import igknighters.commands.teleop.TeleopSwerveSingleAxisCmd;
import igknighters.commands.teleop.TeleopSwerveTraditionalCmd;
// import igknighters.commands.tests.WheelRadiusCharacterization;
// import igknighters.commands.tests.WheelRadiusCharacterization.Direction;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.util.logging.BootupLogger;
import igknighters.util.plumbing.TunableValues;
import java.util.function.DoubleSupplier;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wpilibExt.AllianceSymmetry;

public class DriverController {
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
              return TunableValues.getBoolean("driverAssist", true).value()
                  || DriverStation.isFMSAttached();
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
                    kSwerve.CONSTRAINTS,
                    false)
                .onlyIf(shouldAutoAlign))
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
                    kSwerve.CONSTRAINTS,
                    false)
                .onlyIf(shouldAutoAlign))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));
                    
    this.Y.whileTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Net))
        .whileTrue(
            new TeleopSwerveHeadingCmd(
                    swerve,
                    this,
                    localizer,
                    () -> AllianceSymmetry.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg,
                    new ChassisConstraints(
                        new Constraints(
                            kSwerve.MAX_DRIVE_VELOCITY * 0.5, kSwerve.MAX_DRIVE_ACCELERATION),
                        kSwerve.CONSTRAINTS.rotation()),
                    true)
                .onlyIf(shouldAutoAlign))
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
    // this.LS.onTrue(new WheelRadiusCharacterization(swerve, Direction.COUNTER_CLOCKWISE));

    this.RS.onTrue(operatorTarget.updateTargetCmd(SuperStructureState.ScoreL1, led));

    // // TRIGGERS
    this.LT.and(operatorTarget.hasTarget()).whileTrue(operatorTarget.gotoTargetCmd(this));

    this.RT.onTrue(
        IntakeCommands.bounce(intake)
            .andThen(new ScheduleCommand(IntakeCommands.holdCoral(intake)))
            .withName("IntakeBounce"));

    // DPAD
    this.DPR.onTrue(ClimberCommands.stow(climber));

    // this.DPD.onTrue(ClimberCommands.stage(climber));
    this.DPD.onTrue(
      Commands.parallel(
        ClimberCommands.stage(climber),
        new TeleopSwerveHeadingCmd(swerve, this, localizer, null, null, false)
      )
    );

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

  // Define the buttons on the controller

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
    BootupLogger.bootupLog("Controller " + port + " initialized");
    A = controller.a();
    B = controller.b();
    X = controller.x();
    Y = controller.y();
    LB = controller.leftBumper();
    RB = controller.rightBumper();
    Back = controller.back();
    Start = controller.start();
    LS = controller.leftStick();
    RS = controller.rightStick();
    LT = controller.leftTrigger(0.25);
    RT = controller.rightTrigger(0.25);
    DPR = controller.povRight();
    DPD = controller.povDown();
    DPL = controller.povLeft();
    DPU = controller.povUp();
  }

  private DoubleSupplier deadbandSupplier(DoubleSupplier supplier, double deadband) {
    return () -> {
      double val = supplier.getAsDouble();
      if (Math.abs(val) > deadband) {
        if (val > 0.0) {
          val = (val - deadband) / (1.0 - deadband);
        } else {
          val = (val + deadband) / (1.0 - deadband);
        }
      } else {
        val = 0.0;
      }
      return val;
    };
  }

  /**
   * Right on the stick is positive (axis 4)
   *
   * @return A supplier for the value of the right stick x axis
   */
  public DoubleSupplier rightStickX() {
    return () -> -controller.getRightX();
  }

  /**
   * Right on the stick is positive (axis 4)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the right stick x axis
   */
  public DoubleSupplier rightStickX(double deadband) {
    return deadbandSupplier(rightStickX(), deadband);
  }

  /**
   * Up on the stick is positive (axis 5)
   *
   * @return A supplier for the value of the right stick y axis
   */
  public DoubleSupplier rightStickY() {
    return controller::getRightY;
  }

  /**
   * Up on the stick is positive (axis 5)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the right stick y axis
   */
  public DoubleSupplier rightStickY(double deadband) {
    return deadbandSupplier(rightStickY(), deadband);
  }

  /**
   * Right on the stick is positive (axis 0)
   *
   * @return A supplier for the value of the left stick x axis
   */
  public DoubleSupplier leftStickX() {
    return controller::getLeftX;
  }

  /**
   * Right on the stick is positive (axis 0)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the left stick x axis
   */
  public DoubleSupplier leftStickX(double deadband) {
    return deadbandSupplier(leftStickX(), deadband);
  }

  /**
   * Up on the stick is positive (axis 1)
   *
   * @return A supplier for the value of the left stick y axis
   */
  public DoubleSupplier leftStickY() {
    return () -> -controller.getLeftY();
  }

  /**
   * Up on the stick is positive (axis 1)
   *
   * @param deadband the deadband to apply to the stick
   * @return A supplier for the value of the left stick y axis
   */
  public DoubleSupplier leftStickY(double deadband) {
    return deadbandSupplier(leftStickY(), deadband);
  }

  /**
   * will print warning if this trigger is also bound to a command
   *
   * @param suppressWarning if true will not print warning even if bound to a command
   */
  public DoubleSupplier rightTrigger(boolean suppressWarning) {
    return controller::getRightTriggerAxis;
  }

  /**
   * will print warning if this trigger is also bound to a command
   *
   * @param suppressWarning if true will not print warning even if bound to a command
   */
  public DoubleSupplier leftTrigger(boolean suppressWarning) {
    return controller::getLeftTriggerAxis;
  }

  /**
   * Will rumble both sides of the controller with a magnitude
   *
   * @param magnitude The magnitude to rumble at
   */
  public void rumble(double magnitude) {
    controller.getHID().setRumble(RumbleType.kBothRumble, magnitude);
  }
}
