package igknighters.subsystems.superStructure.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import igknighters.SimCtx;
import sham.ShamIntake;
import sham.ShamMechanism;
import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.UnitSafeControl;
import sham.shamController.UnitSafeControl.ElevatorFeedforward;
import sham.shamController.UnitSafeControl.LinearPIDFeedback;
import sham.shamController.UnitSafeControl.TrapezoidProfile;
import sham.utils.GearRatio;
import wpilibExt.DCMotorExt;

public class ElevatorSim extends Elevator {
  private final ShamMCX shamMCX = new ShamMCX("ElevatorMotor");
  private final ShamMechanism shamMechanism;
   private final ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> elevatorLoop;
  public ElevatorSim(SimCtx simCtx) {
    shamMechanism = new ShamMechanism(
      "ElevatorMechanism",
      new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1), 
      shamMCX, 
      KilogramSquareMeters.of(.2), 
      GearRatio.reduction(25), 
      Friction.of(DCMotor.getKrakenX60Foc(1), Volts.of(1)), 
      MechanismDynamics.forElevator(Pounds.of(35.0), Meters.of(ElevatorConstants.ELEVATOR_MAX_HEIGHT-ElevatorConstants.Elevator_HEIGHT_ABOVE_GROUND)), 
      HardLimits.of(Radians.of(0.0), Radians.of(1.605)),
      0.0, 
      simCtx.robot().timing()
    );
    simCtx.robot().addMechanism(shamMechanism);
    elevatorLoop = ClosedLoop.forCurrentDistance(
        new LinearPIDFeedback<CurrentUnit>(
          Amps.per(Meters).ofNative(ElevatorConstants.KP),
          Amps.per(MetersPerSecond).ofNative(ElevatorConstants.KD)
        ),
        new ElevatorFeedforward<CurrentUnit>(
          Amps.of(ElevatorConstants.KS),
          Amps.per(MetersPerSecond).ofNative(0.0),
          Amps.per(MetersPerSecondPerSecond).ofNative(ElevatorConstants.KA)
        ),
        TrapezoidProfile.forDistance(
          MetersPerSecond.of(0.0),
          MetersPerSecondPerSecond.of(0.0)
        ),
        idk -> Meters.of(1.0)
    );
    shamMCX.configSensorToMechanismRatio(Elevator)
  }

  @Override
  public void gotoPosition(double heightMeters) {
  }

  @Override
  public boolean isAtPosition(double heightMeters, double toleranceMeters) {
    // TODO Auto-generated method stub
    return false;
  }

  public void setNuetralMode(boolean shouldBeCoast) {
    // TODO Auto-generated method stub

  }
}
