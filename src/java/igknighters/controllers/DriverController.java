package igknighters.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.commands.superStructure.StateManager;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;

public class DriverController extends ControllerBase {

  public DriverController(int port, Localizer localizer, Subsystems subsystems) {
    super(port, true);

    StateManager stateManager = new StateManager();

    // disregard null safety for subsystems as it is checked on assignment

    /// FACE BUTTONS
    this.A.onTrue(stateManager.moveTo(subsystems.superStructure, SuperStructureState.IntakeHp));

    this.B.onTrue(stateManager.moveTo(subsystems.superStructure, SuperStructureState.ScoreL4));

    this.X.onTrue(stateManager.moveTo(subsystems.superStructure, SuperStructureState.Processor));

    this.Y.onTrue(stateManager.moveTo(subsystems.superStructure, SuperStructureState.ScoreL4));

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
