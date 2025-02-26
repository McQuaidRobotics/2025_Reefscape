package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.Conv;
import igknighters.subsystems.intake.IntakeConstants.RollerConstants;
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

  public void voltageOut(double voltage) {
    super.controlledLastCycle = true;
    intakeMotor.controlVoltage(Volts.of(voltage));
  }

  public void currentOut(double current) {
    super.controlledLastCycle = true;
    intakeMotor.controlCurrent(Amps.of(current));
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
  }
}
