package sham.shamController;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
import sham.shamController.unitSafeControl.UnitFeedback.PIDFeedback;
import sham.shamController.unitSafeControl.UnitFeedforward;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile;
import sham.shamController.unitSafeControl.UnitTrapezoidProfile.State;
import wpilibExt.MeasureMath;

public class ClosedLoop<OUTPUT extends Unit, INPUT extends Unit> {
  private final PIDFeedback<OUTPUT, INPUT> feedback;
  private final UnitFeedforward<OUTPUT> feedforward;
  private final Optional<UnitTrapezoidProfile<INPUT>> optTrapezoidProfile;

  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      UnitFeedforward<OUTPUT> feedforward,
      Optional<UnitTrapezoidProfile<INPUT>> trapezoidProfile) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
  }

  public static ClosedLoop<VoltageUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      UnitFeedforward<VoltageUnit> feedforward,
      UnitTrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile));
  }

  public static ClosedLoop<VoltageUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback, UnitFeedforward<VoltageUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty());
  }

  public static ClosedLoop<VoltageUnit, AngularVelocityUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<VoltageUnit> feedforward,
      UnitTrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile));
  }

  public static ClosedLoop<VoltageUnit, AngularVelocityUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<VoltageUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty());
  }

  public static ClosedLoop<CurrentUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      UnitFeedforward<CurrentUnit> feedforward,
      UnitTrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile));
  }

  public static ClosedLoop<CurrentUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback, UnitFeedforward<CurrentUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty());
  }

  public static ClosedLoop<CurrentUnit, AngularVelocityUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<CurrentUnit> feedforward,
      UnitTrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile));
  }

  public static ClosedLoop<CurrentUnit, AngularVelocityUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      UnitFeedforward<CurrentUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty());
  }

  public void log(EpilogueBackend logger) {
    feedback.logGains(logger.getNested("feedback"));
    feedforward.logGains(logger.getNested("feedforward"));
    logger.log("profilePresent", optTrapezoidProfile.isPresent());
    if (optTrapezoidProfile.isPresent()) {
      optTrapezoidProfile.get().logConstraints(logger.getNested("profile"));
    }
  }

  public boolean isVelocity() {
    return feedback.getInputUnit() instanceof PerUnit;
  }

  private Measure<OUTPUT> calcFeedForward(Angle position, State<INPUT> state, State<INPUT> step) {
    AngularVelocity currentVelocity;
    AngularVelocity nextVelocity;
    if (isVelocity()) {
      currentVelocity = RadiansPerSecond.of(state.value().baseUnitMagnitude());
      nextVelocity = RadiansPerSecond.of(step.value().baseUnitMagnitude());
    } else {
      currentVelocity = RadiansPerSecond.of(state.slew().baseUnitMagnitude());
      nextVelocity = RadiansPerSecond.of(step.slew().baseUnitMagnitude());
    }
    return MeasureMath.zeroIfNAN(feedforward.calculate(position, currentVelocity, nextVelocity));
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> run(
      Angle position, State<INPUT> state, State<INPUT> goal, Time dt, EpilogueBackend logger) {
    logger.log("position", position);
    logger.log("state", state, State.struct);
    logger.log("goal", goal, State.struct);

    if (optTrapezoidProfile.isPresent()) {
      UnitTrapezoidProfile<INPUT> trapezoidProfile = optTrapezoidProfile.get();
      goal = trapezoidProfile.calculate(state, goal, dt);
      logger.log("step", goal, State.struct);
    }
    var fbResult = feedback.calculate(state.value(), goal.value());
    Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
    Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
    if (goal.slew() != null || isVelocity()) {
      feedforwardOutput = calcFeedForward(position, state, goal);
    } else {
      feedforwardOutput =
          feedforward.calculateStatics(position, Math.signum(feedbackOutput.magnitude()));
    }

    logger.log("feedbackError", fbResult.getSecond());
    logger.log("feedforwardOutput", feedforwardOutput);
    logger.log("feedbackOutput", feedbackOutput);
    return feedbackOutput.plus(feedforwardOutput);
  }
}
