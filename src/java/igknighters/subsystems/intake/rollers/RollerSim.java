package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.intake.IntakeConstants.RollerConstants;
import sham.ShamGamePiece;
import sham.ShamIndexer;
import sham.ShamIntake;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.seasonspecific.Reefscape;
import sham.shamController.ShamMCX;
import sham.utils.GearRatio;
import wpilibExt.AllianceSymmetry;
import wpilibExt.AllianceSymmetry.SymmetryStrategy;
import wpilibExt.DCMotorExt;

public class RollerSim extends Rollers {
  private static final Translation2d[] intakePositions;

  static {
    final Translation2d[] intakePositionsCore = {
      new Translation2d(0.56, 6.7),
      new Translation2d(1.65, 7.5),
      AllianceSymmetry.flip(new Translation2d(0.56, 6.7), SymmetryStrategy.HORIZONTAL),
      AllianceSymmetry.flip(new Translation2d(1.65, 7.5), SymmetryStrategy.HORIZONTAL),
      AllianceSymmetry.flip(new Translation2d(0.56, 6.7), SymmetryStrategy.VERTICAL),
      AllianceSymmetry.flip(new Translation2d(1.65, 7.5), SymmetryStrategy.VERTICAL),
      AllianceSymmetry.flip(new Translation2d(0.56, 6.7), SymmetryStrategy.ROTATIONAL),
      AllianceSymmetry.flip(new Translation2d(1.65, 7.5), SymmetryStrategy.ROTATIONAL),
    };
    intakePositions = new Translation2d[20];
    for (int i = 0; i < 4; i++) {
      int positionsOffset = i * 5;
      Translation2d interpStart = intakePositionsCore[i * 2];
      Translation2d interpEnd = intakePositionsCore[i * 2 + 1];
      intakePositions[positionsOffset] = interpStart;
      intakePositions[positionsOffset + 4] = interpEnd;
      for (int j = 0; j < 4; j++) {
        double t = 0.2 + (0.2 * j);
        intakePositions[positionsOffset + j + 1] = interpStart.interpolate(interpEnd, t);
      }
    }
  }

  private final Debouncer intakeDebouncer = new Debouncer(0.6, DebounceType.kRising);
  private boolean lastAutoEnabled = false;

  private final ShamMechanism intakeMechanism;
  private final ShamMCX intakeMotor;
  private final SimCtx sim;
  private final ShamIntake intake;
  private final ShamIndexer indexer;

  public RollerSim(SimCtx simCtx) {
    intakeMotor = new ShamMCX("IntakeMotor");
    intakeMechanism =
        new ShamMechanism(
            "IntakeMechanism",
            new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1),
            intakeMotor,
            KilogramSquareMeters.of(0.03),
            GearRatio.reduction(RollerConstants.GEAR_RATIO),
            Friction.of(DCMotor.getKrakenX60Foc(1), Volts.of(0.12)),
            MechanismDynamics.zero(),
            HardLimits.unbounded(),
            0,
            simCtx.robot().timing());
    simCtx.robot().addMechanism(intakeMechanism);
    intake =
        simCtx
            .robot()
            .createIntake(
                new Rectangle2d(
                    new Pose2d(15.0 * Conv.INCHES_TO_METERS, 0.0, Rotation2d.kZero),
                    5.0 * Conv.INCHES_TO_METERS,
                    11.0 * Conv.INCHES_TO_METERS),
                Reefscape.ALGAE,
                Reefscape.CORAL);
    indexer = simCtx.robot().getIndexer();
    sim = simCtx;
  }

  public void voltageOut(double voltage) {
    super.controlledLastCycle = true;
    intakeMotor.controlVoltage(Volts.of(voltage));
  }

  public void currentOut(double current) {
    super.controlledLastCycle = true;
    intakeMotor.controlCurrent(Amps.of(current));
  }

  @Override
  public boolean isStalling() {
    return amps < 0.0 && isLaserTripped();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !super.controlledLastCycle) {
      voltageOut(0.0);
    }
    super.controlledLastCycle = false;
    super.volts = intakeMotor.voltage().in(Volts);
    super.amps = intakeMotor.statorCurrent().in(Amps);
    super.radiansPerSecond = intakeMotor.velocity().in(RadiansPerSecond);
    super.laserTripped =
        indexer
            .peekGamePiece()
            .map(gp -> gp.isOfVariant(Reefscape.CORAL) || gp.isOfVariant(Reefscape.ALGAE))
            .orElse(false);
    if (volts < 0.0 && !isLaserTripped()) {
      intake.startIntake();
    } else {
      intake.stopIntake();
    }
    if (volts > -0.01) {
      indexer.removeGamePiece();
    }

    // sham gamepiece intake stuff is broken rn, temporary fix
    boolean nearCoralStation = false;
    Translation2d roboPose = sim.robot().getDriveTrain().getChassisWorldPose().getTranslation();
    for (Translation2d pos : intakePositions) {
      if (roboPose.getDistance(pos) < 0.3) {
        nearCoralStation = true;
        break;
      }
    }
    if (intakeDebouncer.calculate(
        nearCoralStation && !isLaserTripped() && intake.isIntakeRunning())) {
      indexer.insertGamePiece(new ShamGamePiece(Reefscape.CORAL, sim.arena()));
    }
    if (!lastAutoEnabled && DriverStation.isAutonomousEnabled()) {
      System.out.println("Auto enabled");
      indexer.insertGamePiece(new ShamGamePiece(Reefscape.CORAL, sim.arena()));
    }
    lastAutoEnabled = DriverStation.isAutonomousEnabled();
  }
}
