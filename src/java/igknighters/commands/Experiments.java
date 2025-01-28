package igknighters.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableDouble;
import monologue.Monologue;
import wpilibExt.DCMotorExt;

public class Experiments {

  public static Command dcmotorTestCmd(DCMotor simpleMotor) {
    final DCMotorExt motor = new DCMotorExt(simpleMotor, 1);
    final TunableDouble volts = TunableValues.getDouble("dcmotorTest/volts", -12.0);
    final TunableDouble rpmPercent = TunableValues.getDouble("dcmotorTest/rpmPercent", -0.5);
    final TunableDouble statorLimit = TunableValues.getDouble("dcmotorTest/statorLimit", 90);
    final TunableDouble supplyLimit = TunableValues.getDouble("dcmotorTest/supplyLimit", 60);
    Monologue.log("/Tunables/dcmotorTest/motor", motor);
    return Commands.run(
            () -> {
              final Voltage inputVoltage = Volts.of(volts.value());
              final AngularVelocity rpm = motor.freeSpeed().times(rpmPercent.value());
              final var limitedStator =
                  Monologue.log(
                      "/Tunables/dcmotorTest/outputLimtedStatorCurrent",
                      motor
                          .getCurrent(
                              rpm,
                              inputVoltage,
                              Amps.of(supplyLimit.value()),
                              Amps.of(statorLimit.value()))
                          .in(Amps));
              final var stator =
                  Monologue.log(
                      "/Tunables/dcmotorTest/outputStatorCurrent",
                      motor.getCurrent(rpm, inputVoltage).in(Amps));
              Monologue.log(
                  "/Tunables/dcmotorTest/outputLimitedSupplyCurrent",
                  motor.getSupplyCurrent(rpm, inputVoltage, Amps.of(limitedStator)).in(Amps));
              Monologue.log(
                  "/Tunables/dcmotorTest/outputSupplyCurrent",
                  motor.getSupplyCurrent(rpm, inputVoltage, Amps.of(stator)).in(Amps));
            })
        .ignoringDisable(true);
  }
}
