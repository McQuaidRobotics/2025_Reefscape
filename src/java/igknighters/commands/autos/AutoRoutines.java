package igknighters.commands.autos;

import static igknighters.commands.autos.Waypoints.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.commands.IntakeCommands;
import igknighters.commands.SuperStructureCommands;
import igknighters.commands.SuperStructureCommands.MoveOrder;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.superStructure.SuperStructureState;
import java.util.function.Supplier;

public class AutoRoutines extends AutoCommands {

  public AutoRoutines(Subsystems subsystems, Localizer localizer, AutoFactory factory) {
    super(factory, subsystems, localizer);

    // if (Robot.isSimulation()) {
    //   new Trigger(DriverStation::isAutonomousEnabled)
    //       .onTrue(
    //           Commands.waitSeconds(15.3)
    //               .andThen(() -> DriverStationSim.setEnabled(false))
    //               .withName("Simulated Auto Ender"));
    // }
  }

  public Supplier<Command> trajTest(String trajName) {
    return () ->
        Commands.sequence(factory.resetOdometry(trajName), factory.trajectoryCmd(trajName));
  }

  @SuppressWarnings("unchecked")
  public Trajectory<SwerveSample> flipTrajectory(boolean leftSide, Waypoints start, Waypoints end) {
    Trajectory<SwerveSample> rawTraj =
        (Trajectory<SwerveSample>) factory.cache().loadTrajectory(start.to(end)).orElseThrow();
    if (leftSide) {
      return rawTraj;
    } else {
      return rawTraj.flipped(Flipper.MIRRORED_X);
    }
  }

  public Command L4ToAlgaeFarMid(boolean leftSide) {

    Trajectory<?> trajectory = flipTrajectory(leftSide, FarMid_L, FarMid_C);
    return Commands.parallel(
            Commands.runOnce(() -> Commands.print("L4ToAlgaeFarMid:3")),
            factory
                .trajectoryCmd(trajectory)
                .andThen(
                    SwerveCommands.moveToSimple(
                        swerve, localizer, trajectory.getFinalPose(leftSide).get())),
            Commands.sequence(
                Commands.waitSeconds(2),
                SuperStructureCommands.holdAt(
                    superStructure, SuperStructureState.AlgaeL2, MoveOrder.ELEVATOR_FIRST)),
            IntakeCommands.intakeAlgae(super.intake))
        .withTimeout(2)
        .withName("L4ToAlgaeFarMid");
  }

  public Command StartingMiddleToFarMid_L(boolean leftSide) {
    Trajectory<?> trajectory = flipTrajectory(leftSide, StartingMiddle, FarMid_L);

    return Commands.parallel(
            Commands.sequence(
                    Commands.print("StartingMiddleToFarMid_L:)"),
                    factory.resetOdometry(trajectory),
                    factory.trajectoryCmd(trajectory))
                .andThen(
                    SwerveCommands.moveToSimple(
                        swerve, localizer, trajectory.getFinalPose(leftSide).get())),
            SuperStructureCommands.holdAt(
                superStructure, SuperStructureState.ScoreL4, MoveOrder.ELEVATOR_FIRST),
            IntakeCommands.holdCoral(intake))
        .until(localizer.near(trajectory.getFinalPose(leftSide).get().getTranslation(), 0.03))
        .andThen(IntakeCommands.expel(intake).withTimeout(0.3))
        .withName("StartingMiddleToFarMid_L");
  }

  public Command algaeFarMid_CToBarge_R(boolean leftSide) {
    Trajectory<?> trajectory = flipTrajectory(leftSide, FarMid_C, Barge_R);
    return Commands.parallel(
            Commands.runOnce(() -> Commands.print("ALGAE FAR MID C TO BARGE:(")),
            IntakeCommands.holdAlgae(super.intake),
            factory
                .trajectoryCmd(trajectory)
                .andThen(
                    SwerveCommands.moveToSimple(
                            swerve, localizer, trajectory.getFinalPose(leftSide).get())
                        .until(
                            localizer.near(
                                new Translation2d(
                                    trajectory.getFinalPose(leftSide).get().getX(),
                                    trajectory.getFinalPose(leftSide).get().getY()),
                                0.1))))
        .andThen(
            Commands.parallel(
                SuperStructureCommands.holdAt(superStructure, SuperStructureState.Net_FLICKED),
                Commands.sequence(
                    IntakeCommands.holdAlgae(super.intake).withTimeout(0.14),
                    IntakeCommands.expel(super.intake).withTimeout(0.25))))
        // .until(IntakeCommands.isHolding(intake, Holding.NONE))
        .withName("algaeFarMidToBarge");
  }

  public Supplier<Command> algaeBarge(boolean leftSide) {
    return () ->
        SuperStructureCommands.home(superStructure, true)
            .andThen(StartingMiddleToFarMid_L(leftSide))
            .andThen(L4ToAlgaeFarMid(leftSide))
            .andThen(algaeFarMid_CToBarge_R(leftSide));
  }

  @FunctionalInterface
  public interface DualSideAuto {
    Command generate(boolean leftSide);
  }

  public static void addCmd(AutoChooser chooser, String name, DualSideAuto auto) {
    chooser.addCmd(name + " Left", () -> auto.generate(true));
    chooser.addCmd(name + " Right", () -> auto.generate(false));
  }

  public Command test(boolean leftSide) {
    return newAuto("test", leftSide)
        .addTrajectories(
            StartingOutside, FarLeft_R, Intake, CloseLeft_L, Intake, CloseLeft_R, Intake, FarLeft_L)
        .build();
  }

  public Command testMove(boolean leftSide) {
    return newAuto("testMove", leftSide)
        .addTrajectoriesMoveOnly(
            StartingOutside, FarLeft_R, Intake, CloseLeft_L, Intake, CloseLeft_R, Intake)
        .build();
  }

  public Command threeL4s(boolean leftSide) {
    return newAuto("threeL4", leftSide).addTrajectories(StartingOutside, FarLeft_R, Intake).build();
  }
}
