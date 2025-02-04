package igknighters.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.commands.superStructure.StateManager;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;
import igknighters.util.logging.BootupLogger;
import java.util.function.DoubleSupplier;

public class DriverController {
  // Define the bindings for the controller
  @SuppressWarnings("unused")
  public void bind(final Localizer localizer, final Subsystems subsystems) {
    final var swerve = subsystems.swerve;
    final var vision = subsystems.vision;
    final var led = subsystems.led;

    final StateManager stateManager = new StateManager(subsystems.superStructure);

    /// FACE BUTTONS
    this.A.onTrue(stateManager.holdAt(SuperStructureState.IntakeHp));

    this.B.onTrue(stateManager.holdAt(SuperStructureState.ScoreL4));

    this.X.onTrue(stateManager.holdAt(SuperStructureState.Processor));

    this.Y.whileTrue(stateManager.holdAt(SuperStructureState.ScoreL2));

    // BUMPER
    this.RB.onTrue(Commands.none());

    this.LB.onTrue(Commands.none());

    // CENTER BUTTONS
    this.Back.onTrue(Commands.none());

    this.Start.onTrue(SwerveCommands.orientGyro(swerve, localizer));

    // STICKS
    this.LS.onTrue(Commands.none());

    this.RS.onTrue(Commands.none());

    // TRIGGERS
    this.LT.onTrue(Commands.none());

    this.RT.onTrue(Commands.none());

    // DPAD
    this.DPR.onTrue(Commands.none());

    this.DPD.onTrue(Commands.none());

    this.DPL.onTrue(Commands.none());

    this.DPU.onTrue(Commands.none());
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
    return () -> -controller.getLeftX();
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
    return controller::getLeftY;
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
