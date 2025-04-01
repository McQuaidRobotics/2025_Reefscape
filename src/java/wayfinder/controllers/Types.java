package wayfinder.controllers;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.ProceduralStructGenerator;

public class Types {
  public record State(double position, double velocity) implements StructSerializable {
    public static final State kZero = new State(0.0, 0.0);

    public static final Struct<State> struct = ProceduralStructGenerator.genRecord(State.class);
  }

  public record Constraints(double maxVelocity, double maxAcceleration)
      implements StructSerializable {

    public static final Struct<Constraints> struct =
        ProceduralStructGenerator.genRecord(Constraints.class);

    public Constraints min(Constraints other) {
      return new Constraints(
          Math.min(maxVelocity, other.maxVelocity),
          Math.min(maxAcceleration, other.maxAcceleration));
    }
  }

  public record ChassisConstraints(Constraints translation, Constraints rotation)
      implements StructSerializable {
    public static final Struct<ChassisConstraints> struct =
        ProceduralStructGenerator.genRecord(ChassisConstraints.class);

    public ChassisConstraints min(ChassisConstraints other) {
      return new ChassisConstraints(
          translation.min(other.translation), rotation.min(other.rotation));
    }
  }

  public interface Controller<MEASUREMENT, RATE, TARGET, LIMITS> {
    public RATE calculate(
        double period,
        MEASUREMENT measurement,
        RATE measurementRate,
        TARGET target,
        LIMITS constraints);

    public void reset(MEASUREMENT measurement, RATE measurementRate, TARGET target);

    public boolean isDone(MEASUREMENT measurement, TARGET target);
  }

  public interface Receiver<RATE, LIMITS> {
    public void control(RATE rate, LIMITS constraints);
  }

  public static class ControllerSequence<MEASUREMENT, RATE, TARGET, LIMITS>
      implements Controller<MEASUREMENT, RATE, TARGET, LIMITS> {
    private final Controller<MEASUREMENT, RATE, TARGET, LIMITS>[] controllers;
    private int currentController = 0;

    @SuppressWarnings("unchecked")
    public ControllerSequence(Controller<MEASUREMENT, RATE, TARGET, LIMITS>... controllers) {
      this.controllers = controllers;
    }

    @Override
    public RATE calculate(
        double period,
        MEASUREMENT measurement,
        RATE measurementRate,
        TARGET target,
        LIMITS constraints) {
      final var controller = controllers[currentController];
      RATE rate = controller.calculate(period, measurement, measurementRate, target, constraints);
      if (controller.isDone(measurement, target)) {
        currentController = Math.min(currentController + 1, controllers.length - 1);
        controllers[currentController].reset(measurement, measurementRate, target);
      }
      return rate;
    }

    @Override
    public void reset(MEASUREMENT measurement, RATE measurementRate, TARGET target) {
      currentController = 0;
      for (Controller<MEASUREMENT, RATE, TARGET, LIMITS> controller : controllers) {
        controller.reset(measurement, measurementRate, target);
      }
    }

    @Override
    public boolean isDone(MEASUREMENT measurement, TARGET target) {
      return currentController == controllers.length - 1
          && controllers[currentController].isDone(measurement, target);
    }
  }

  public abstract static class WrapperController<MEASUREMENT, RATE, TARGET, LIMITS>
      implements Controller<MEASUREMENT, RATE, TARGET, LIMITS> {
    private final Controller<MEASUREMENT, RATE, TARGET, LIMITS> controller;

    public WrapperController(Controller<MEASUREMENT, RATE, TARGET, LIMITS> controller) {
      this.controller = controller;
    }

    @Override
    public RATE calculate(
        double period,
        MEASUREMENT measurement,
        RATE measurementRate,
        TARGET target,
        LIMITS constraints) {
      return controller.calculate(period, measurement, measurementRate, target, constraints);
    }

    @Override
    public void reset(MEASUREMENT measurement, RATE measurementRate, TARGET target) {
      controller.reset(measurement, measurementRate, target);
    }

    @Override
    public boolean isDone(MEASUREMENT measurement, TARGET target) {
      return controller.isDone(measurement, target);
    }
  }
}
