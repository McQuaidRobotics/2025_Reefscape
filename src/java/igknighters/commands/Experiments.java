package igknighters.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.jni.ArmFeedforwardJNI;
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

  public static Command armFFTestCmd() {
    final TunableDouble ks = TunableValues.getDouble("armff/ks", 0.0);
    final TunableDouble kv = TunableValues.getDouble("armff/kv", 0.0);
    final TunableDouble ka = TunableValues.getDouble("armff/ka", 0.0);
    final TunableDouble kg = TunableValues.getDouble("armff/kg", 0.0);
    final TunableDouble currentAngle = TunableValues.getDouble("armff/currentAngle", 0.0);
    final TunableDouble currentVelocity = TunableValues.getDouble("armff/currentVelocity", 0.0);
    final TunableDouble nextVelocity = TunableValues.getDouble("armff/nextVelocity", 0.0);
    final TunableDouble dt = TunableValues.getDouble("armff/dt", 0.02);

    return Commands.run(
            () -> {
              Monologue.log(
                  "/Tunables/armff/output",
                  ArmFeedforwardJNI.calculate(
                      ks.value(),
                      kv.value(),
                      ka.value(),
                      kg.value(),
                      currentAngle.value(),
                      currentVelocity.value(),
                      nextVelocity.value(),
                      dt.value()));
            })
        .ignoringDisable(true);
  }

  // public static Command testScoringTime(SuperStructureManager ss) {
  //   Timer timer = new Timer();
  //   return Commands.sequence(
  //       ss.moveTo(SuperStructureState.ScoreL2),
  //       Commands.runOnce(() -> Monologue.log("bleh", "pre-amble")),
  //       Commands.runOnce(() -> timer.restart()),
  //       ss.moveTo(SuperStructureState.ScoreL4),
  //       Commands.waitSeconds(0.15),
  //       Commands.runOnce(() -> Monologue.log("scoreHalfTime", timer.get())),
  //       ss.moveTo(SuperStructureState.ScoreL2),
  //       Commands.runOnce(() -> timer.stop()),
  //       Commands.runOnce(() -> Monologue.log("scoreTime", timer.get())));
  // }
}
