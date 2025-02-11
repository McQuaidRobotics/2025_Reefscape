package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import sham.ShamIndexer;
import sham.ShamIntake;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.seasonspecific.Reefscape;
import sham.shamController.ShamMCX;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class RollerSim extends Rollers {
  private final ShamMechanism intakeMechanism;
  private final ShamMCX intakeMotor;
  private final ShamIntake intake;
  private final ShamIndexer indexer;

  public RollerSim(SimCtx simCtx) {
    super(DCMotor.getKrakenX60(1).withReduction(RollerConstants.GEAR_RATIO));
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
  }

  public void setVoltage(double voltage) {
    intakeMotor.controlVoltage(Volts.of(voltage));
  }

  public void setCurrent(double current) {
    intakeMotor.controlCurrent(Amps.of(current));
  }

  @Override
  public boolean hasAlgae() {
    return indexer.peekGamePiece().map(gp -> gp.isOfVariant(Reefscape.ALGAE)).orElse(false);
  }

  @Override
  public boolean hasCoral() {
    return indexer.peekGamePiece().map(gp -> gp.isOfVariant(Reefscape.CORAL)).orElse(false);
  }

  @Override
  public void periodic() {
    super.volts = intakeMotor.voltage().in(Volts);
    super.current = intakeMotor.statorCurrent().in(Amps);
    super.radiansPerSecond = intakeMotor.velocity().in(RadiansPerSecond);
    super.hasAlgae = hasAlgae();
    super.hasCoral = hasCoral();
    if (volts < 0.0 && !hasCoral() && !hasAlgae()) {
      intake.startIntake();
    } else {
      intake.stopIntake();
    }
    if (volts > 0.0) {
      indexer.removeGamePiece();
    }
  }
}
