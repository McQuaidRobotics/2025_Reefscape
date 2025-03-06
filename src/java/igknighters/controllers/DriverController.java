package igknighters.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.ClimberCommands;
import igknighters.commands.IntakeCommands;
// import igknighters.commands.LEDCommands;
import igknighters.commands.OperatorTarget;
import igknighters.commands.SuperStructureCommands;
import igknighters.commands.SwerveCommands;
import igknighters.commands.teleop.TeleopSwerveHeadingCmd;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.intake.Intake.Holding;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.subsystems.swerve.SwerveConstants.kSwerve;
import igknighters.util.logging.BootupLogger;
import java.util.function.DoubleSupplier;
import wpilibExt.AllianceFlipper;

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

    /// FACE BUTTONS
    this.A.whileTrue(
            SuperStructureCommands.holdAt(superStructure, SuperStructureState.IntakeHp)
                .alongWith(
                    IntakeCommands.intakeCoral(subsystems.intake),
                    new TeleopSwerveHeadingCmd(
                        swerve,
                        this,
                        localizer,
                        () -> {
                          final double angle = 54.0;
                          if (false) {
                            return AllianceFlipper.isBlue()
                                ? Rotation2d.fromDegrees(180 - angle)
                                : Rotation2d.fromDegrees(angle);
                          } else {
                            return AllianceFlipper.isBlue()
                                ? Rotation2d.fromDegrees(-(180 - angle))
                                : Rotation2d.fromDegrees(-angle);
                          }
                        },
                        kSwerve.CONSTRAINTS))
                .until(subsystems.intake.isHolding(Holding.CORAL)))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    this.B.whileTrue(
            SuperStructureCommands.holdAt(superStructure, SuperStructureState.Processor)
                .alongWith(
                    new TeleopSwerveHeadingCmd(
                        swerve,
                        this,
                        localizer,
                        () -> AllianceFlipper.isBlue() ? Rotation2d.kCW_Pi_2 : Rotation2d.kCCW_Pi_2,
                        kSwerve.CONSTRAINTS)))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    this.X.onTrue(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    this.Y.whileTrue(
            SuperStructureCommands.holdAt(superStructure, SuperStructureState.Net)
                .alongWith(
                    new TeleopSwerveHeadingCmd(
                        swerve,
                        this,
                        localizer,
                        () -> AllianceFlipper.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg,
                        kSwerve.CONSTRAINTS)))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    // BUMPER
    this.RB.onTrue(IntakeCommands.expel(intake).withTimeout(0.4));

    this.LB
        .whileTrue(operatorTarget.gotoSuperStructureTargetCmd())
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    // CENTER BUTTONS
    this.Back.onTrue(SuperStructureCommands.home(superStructure, true));

    this.Start.onTrue(SwerveCommands.orientGyro(swerve, localizer));

    // STICKS
    this.LS.onTrue(Commands.none());

    this.RS.onTrue(Commands.none());

    // // TRIGGERS
    this.LT
        .and(operatorTarget.hasTarget())
        .whileTrue(operatorTarget.gotoTargetCmd(localizer))
        .onFalse(SuperStructureCommands.holdAt(superStructure, SuperStructureState.Stow));

    this.RT
        .and(operatorTarget.superStructureAtSetpoint())
        .and(new Trigger(intake.isHolding(Holding.NONE)).negate())
        .onTrue(IntakeCommands.expel(intake).until(intake.isHolding(Holding.NONE)));

    // DPAD
    this.DPR.whileTrue(ClimberCommands.stow(climber));

    this.DPD.whileTrue(ClimberCommands.stage(climber));

    this.DPL.onTrue(
        climber.run(() -> climber.voltageOut(-3.0)).finallyDo(() -> climber.voltageOut(0.0)));

    this.DPU.whileTrue(ClimberCommands.climb(climber));
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
