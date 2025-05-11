package wayfinder.controllers;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class Framework {
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

  public interface Acceptor<RATE, LIMITS> {
    public void control(RATE rate, LIMITS constraints);

    public static <RATE, LIMITS> Acceptor<RATE, LIMITS> of(BiConsumer<RATE, LIMITS> consumer) {
      return new Acceptor<RATE, LIMITS>() {
        @Override
        public void control(RATE rate, LIMITS constraints) {
          consumer.accept(rate, constraints);
        }
      };
    }
  }

  public interface Assessor<MEASUREMENT, RATE> {
    public MEASUREMENT getMeasurement();

    public RATE getRate();

    public static <MEASUREMENT, RATE> Assessor<MEASUREMENT, RATE> of(
        Supplier<MEASUREMENT> measurementSupplier, Supplier<RATE> rateSupplier) {
      return new Assessor<MEASUREMENT, RATE>() {
        @Override
        public MEASUREMENT getMeasurement() {
          return measurementSupplier.get();
        }

        @Override
        public RATE getRate() {
          return rateSupplier.get();
        }
      };
    }
  }

  public static class Stack<MEASUREMENT, RATE, TARGET, LIMITS> {
    private final Assessor<MEASUREMENT, RATE> assessor;
    private final Acceptor<RATE, LIMITS> acceptor;
    private final Controller<MEASUREMENT, RATE, TARGET, LIMITS> controller;

    public Stack(
        Assessor<MEASUREMENT, RATE> assessor,
        Controller<MEASUREMENT, RATE, TARGET, LIMITS> controller,
        Acceptor<RATE, LIMITS> acceptor) {
      this.assessor = assessor;
      this.acceptor = acceptor;
      this.controller = controller;
    }

    public void reset(TARGET target) {
      controller.reset(assessor.getMeasurement(), assessor.getRate(), target);
    }

    public void control(double period, TARGET target, LIMITS constraints) {
      final RATE rate =
          controller.calculate(
              period, assessor.getMeasurement(), assessor.getRate(), target, constraints);
      acceptor.control(rate, constraints);
    }
  }

  public static class ControllerSequence<MEASUREMENT, RATE, TARGET, LIMITS>
      implements Controller<MEASUREMENT, RATE, TARGET, LIMITS> {
    private final Controller<MEASUREMENT, RATE, TARGET, LIMITS>[] controllers;
    private int currentController = 0;

    @SafeVarargs
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
