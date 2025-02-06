package wayfinder.controllers;

import wayfinder.controllers.Types.State;

public class DynamicTrapezoidProfile {
  private DynamicTrapezoidProfile() {
    throw new UnsupportedOperationException(
        "DynamicTrapezoidProfile is a utility class and cannot be instantiated");
  }

  private static boolean shouldFlipAcceleration(double initialPosition, double goalPosition) {
    return initialPosition > goalPosition;
  }

  private static double direct(double number, boolean forward) {
    double direction = forward ? -1.0 : 1.0;
    return number * direction;
  }

  public static State calculate(
      double period,
      double currentPos,
      double currentVel,
      double goalPos,
      double goalVel,
      double maxVel,
      double maxAccel) {
    boolean direction = shouldFlipAcceleration(currentPos, goalPos);
    currentPos = direct(currentPos, direction);
    currentVel = direct(currentVel, direction);
    goalPos = direct(goalPos, direction);
    goalVel = direct(goalVel, direction);

    if (currentVel > maxVel) {
      currentVel = maxVel;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    double cutoffBegin = currentVel / maxAccel;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * maxAccel / 2.0;

    double cutoffEnd = goalVel / maxAccel;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * maxAccel / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one
    double fullTrapezoidDist = cutoffDistBegin + (goalPos - currentPos) + cutoffDistEnd;
    double accelerationTime = maxVel / maxAccel;

    double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * maxAccel;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
      accelerationTime = Math.sqrt(fullTrapezoidDist / maxAccel);
      fullSpeedDist = 0;
    }

    double endAccel = accelerationTime - cutoffBegin;
    double endFullSpeed = endAccel + fullSpeedDist / maxVel;
    double endDecel = endFullSpeed + accelerationTime - cutoffEnd;

    double resultPos = currentPos;
    double resultVel = currentVel;

    if (period < endAccel) {
      resultVel += period * maxAccel;
      resultPos += (currentVel + period * maxAccel / 2.0) * period;
    } else if (period < endFullSpeed) {
      resultVel = maxVel;
      resultPos +=
          (currentVel + endAccel * maxAccel / 2.0) * endAccel + maxVel * (period - endAccel);
    } else if (period <= endDecel) {
      resultVel = goalVel + (endDecel - period) * maxAccel;
      double timeLeft = endDecel - period;
      resultPos = goalPos - (goalVel + timeLeft * maxAccel / 2.0) * timeLeft;
    } else {
      resultPos = goalPos;
      resultVel = goalVel;
    }

    return new State(direct(resultPos, direction), direct(resultVel, direction));
  }

  public static State calculate(
      double period, State current, State goal, double maxVel, double maxAccel) {
    return calculate(
        period,
        current.position(),
        current.velocity(),
        goal.position(),
        goal.velocity(),
        maxVel,
        maxAccel);
  }
}
