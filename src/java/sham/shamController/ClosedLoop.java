package sham.shamController;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
import java.util.function.Function;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward;
import sham.shamController.unitSafeControl.UnitFeedforward.ArmFeedforward;
import sham.shamController.unitSafeControl.UnitFeedforward.ElevatorFeedforward;
import sham.shamController.unitSafeControl.UnitFeedforward.ElevatorFeedforwardAngularAdapter;
import sham.shamController.unitSafeControl.UnitFeedforward.SimpleFeedforward;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile.State;

public class ClosedLoop<OUTPUT extends Unit, INPUT extends Unit, INPUT_DIMENSION extends Unit> {
  private final PIDFeedback<OUTPUT, INPUT> feedback;
  private final UnitFeedforward<OUTPUT, INPUT_DIMENSION> feedforward;
  private final Optional<UnitTrapezoidProfile<INPUT>> optTrapezoidProfile;
  private final boolean useFeedbackSign;
  private final Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance;

  private State<INPUT> lastStep = null;

  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      UnitFeedforward<OUTPUT, INPUT_DIMENSION> feedforward,
      Optional<UnitTrapezoidProfile<INPUT>> trapezoidProfile,
      boolean useFeedbackSign) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
    this.useFeedbackSign = useFeedbackSign;
    this.angleToDistance = angle -> Meters.zero();
  }

  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      UnitFeedforward<OUTPUT, INPUT_DIMENSION> feedforward,
      Optional<UnitTrapezoidProfile<INPUT>> trapezoidProfile,
      boolean useFeedbackSign,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
    this.useFeedbackSign = useFeedbackSign;
    this.angleToDistance = angleToDistance;
  }

  public static ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      UnitFeedforward<VoltageUnit, AngleUnit> feedforward,
      UnitTrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      UnitFeedforward<VoltageUnit, AngleUnit> feedforward,
      boolean useFeedbackSign) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), useFeedbackSign);
  }

  public static ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<VoltageUnit, AngleUnit> feedforward,
      UnitTrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<VoltageUnit, AngleUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false);
  }

  public static ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      UnitFeedforward<CurrentUnit, AngleUnit> feedforward,
      UnitTrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      UnitFeedforward<CurrentUnit, AngleUnit> feedforward,
      boolean useFeedbackSign) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), useFeedbackSign);
  }

  public static ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<CurrentUnit, AngleUnit> feedforward,
      UnitTrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<CurrentUnit, AngleUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false);
  }

  public static ClosedLoop<VoltageUnit, DistanceUnit, DistanceUnit> forVoltageDistance(
      PIDFeedback<VoltageUnit, DistanceUnit> feedback,
      UnitFeedforward<VoltageUnit, DistanceUnit> feedforward,
      UnitTrapezoidProfile<DistanceUnit> trapezoidProfile,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.of(trapezoidProfile), false, angleToDistance);
  }

  public static ClosedLoop<VoltageUnit, DistanceUnit, DistanceUnit> forVoltageDistance(
      PIDFeedback<VoltageUnit, DistanceUnit> feedback,
      UnitFeedforward<VoltageUnit, DistanceUnit> feedforward,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false, angleToDistance);
  }

  public static ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> forCurrentDistance(
      PIDFeedback<CurrentUnit, DistanceUnit> feedback,
      UnitFeedforward<CurrentUnit, DistanceUnit> feedforward,
      UnitTrapezoidProfile<DistanceUnit> trapezoidProfile,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.of(trapezoidProfile), false, angleToDistance);
  }

  public static ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> forCurrentDistance(
      PIDFeedback<CurrentUnit, DistanceUnit> feedback,
      UnitFeedforward<CurrentUnit, DistanceUnit> feedforward,
      boolean useFeedbackSign,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.empty(), useFeedbackSign, angleToDistance);
  }

  public void log(EpilogueBackend logger) {
    feedback.logGains(logger.getNested("feedback"));
    feedforward.logGains(logger.getNested("feedforward"));
    if (optTrapezoidProfile.isPresent()) {
      optTrapezoidProfile.get().logConstraints(logger.getNested("profile"));
    }
    logger.log("useFeedbackSign", useFeedbackSign);
  }

  public boolean isVelocity() {
    return feedback.getInputUnit() instanceof PerUnit;
  }

  private State<DistanceUnit> angleToDistanceState(State<AngleUnit> state) {
    final VelocityUnit<DistanceUnit> velocityUnit = VelocityUnit.combine(Meters, Seconds);
    final double ratio = angleToDistance.apply(Radians.of(1)).baseUnitMagnitude();
    return new State<>(
        angleToDistance.apply(state.value()),
        velocityUnit.of(state.slew().baseUnitMagnitude() * ratio));
  }

  private State<LinearVelocityUnit> angularVelocityToLinearVelocityState(
      State<AngularVelocityUnit> state) {
    final double ratio = angleToDistance.apply(Radians.of(1)).baseUnitMagnitude();
    final VelocityUnit<LinearVelocityUnit> velocityUnit =
        VelocityUnit.combine(MetersPerSecond, Seconds);
    return new State<>(
        MetersPerSecond.of(state.value().baseUnitMagnitude() * ratio),
        velocityUnit.of(state.slew().baseUnitMagnitude() * ratio));
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> runPosition(
      Angle position,
      State<AngleUnit> state,
      State<AngleUnit> goal,
      Time dt,
      EpilogueBackend logger) {
    if (isVelocity()) {
      throw new UnsupportedOperationException("Velocity not supported for `runAngle`");
    }
    if (feedback.getInputUnit() instanceof DistanceUnit) {
      return run(
          (Measure<INPUT_DIMENSION>) angleToDistance.apply(position),
          (State<INPUT>) angleToDistanceState(state),
          (State<INPUT>) angleToDistanceState(goal),
          dt,
          logger);
    } else {
      return run(
          (Measure<INPUT_DIMENSION>) position,
          (State<INPUT>) state,
          (State<INPUT>) goal,
          dt,
          logger);
    }
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> runVelocity(
      Angle position,
      State<AngularVelocityUnit> state,
      State<AngularVelocityUnit> goal,
      Time dt,
      EpilogueBackend logger) {
    if (!isVelocity()) {
      throw new UnsupportedOperationException("Position not supported for `runVelocity`");
    }
    if (feedback.getInputUnit() instanceof DistanceUnit) {
      return run(
          (Measure<INPUT_DIMENSION>) angleToDistance.apply(position),
          (State<INPUT>) angularVelocityToLinearVelocityState(state),
          (State<INPUT>) angularVelocityToLinearVelocityState(goal),
          dt,
          logger);
    } else {
      return run(
          (Measure<INPUT_DIMENSION>) position,
          (State<INPUT>) state,
          (State<INPUT>) goal,
          dt,
          logger);
    }
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> run(
      Measure<INPUT_DIMENSION> position,
      State<INPUT> state,
      State<INPUT> goal,
      Time dt,
      EpilogueBackend logger) {
    logger.log("position", position);
    logger.log("state", state, State.struct);
    logger.log("goal", goal, State.struct);
    boolean isVelocity = feedback.getInputUnit() instanceof PerUnit;
    if (optTrapezoidProfile.isPresent()) {
      logger.log("profilePresent", true);
      var trapezoidProfile = optTrapezoidProfile.get();
      if (lastStep == null) {
        lastStep = state;
      }
      State<INPUT> step = trapezoidProfile.calculate(lastStep, goal, dt);
      State<INPUT> capturedLastStep = lastStep;
      lastStep = step;
      logger.log("step", step, State.struct);
      Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
      if (feedforward.getClass().equals(SimpleFeedforward.class)) {
        var flywheelFF = (SimpleFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? flywheelFF.calculate(
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : flywheelFF.calculate(
                    RadiansPerSecond.of(capturedLastStep.slew().baseUnitMagnitude()),
                    RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ElevatorFeedforward.class)) {
        var elevatorFF = (ElevatorFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? elevatorFF.calculate(
                    MetersPerSecond.of(step.value().baseUnitMagnitude()),
                    MetersPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : elevatorFF.calculate(
                    MetersPerSecond.of(capturedLastStep.slew().baseUnitMagnitude()),
                    MetersPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ElevatorFeedforwardAngularAdapter.class)) {
        var elevatorFF = (ElevatorFeedforwardAngularAdapter<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? elevatorFF.calculate(
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : elevatorFF.calculate(
                    RadiansPerSecond.of(capturedLastStep.slew().baseUnitMagnitude()),
                    RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ArmFeedforward.class)) {
        var armFF = (ArmFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : armFF.calculate(
                    Radians.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(capturedLastStep.slew().baseUnitMagnitude()),
                    RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else {
        throw new UnsupportedOperationException("Feedforward type not supported");
      }
      logger.log("feedforwardOutput", feedforwardOutput);
      var fbResult = feedback.calculate(state.value(), step.value());
      Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
      logger.log("feedbackOutput", feedbackOutput);
      logger.log("feedbackError", fbResult.getSecond());
      return feedbackOutput.plus(feedforwardOutput);
    } else {
      // Measure<OUTPUT> feedbackOutput = feedback.calculate(state.value(), goal.value());
      var fbResult = feedback.calculate(state.value(), goal.value());
      Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
      logger.log("feedbackError", fbResult.getSecond());
      Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
      double velocitySign =
          useFeedbackSign
              ? 0.00001 * Math.signum(feedbackOutput.baseUnitMagnitude())
              : 0.00001
                  * Math.signum(
                      goal.value().baseUnitMagnitude() - state.value().baseUnitMagnitude());
      if (feedforward.getClass().equals(SimpleFeedforward.class)) {
        var flywheelFF = (SimpleFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                flywheelFF.calculate(RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ElevatorFeedforward.class)) {
        var elevatorFF = (ElevatorFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(MetersPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ElevatorFeedforwardAngularAdapter.class)) {
        var elevatorFF = (ElevatorFeedforwardAngularAdapter<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ArmFeedforward.class)) {
        var armFF = (ArmFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else {
        throw new UnsupportedOperationException("Feedforward type not supported");
      }
      logger.log("feedforwardOutput", feedforwardOutput);
      logger.log("feedbackOutput", feedbackOutput);
      return feedbackOutput.plus(feedforwardOutput);
    }
  }

  public void reset() {
    lastStep = null;
  }
}
