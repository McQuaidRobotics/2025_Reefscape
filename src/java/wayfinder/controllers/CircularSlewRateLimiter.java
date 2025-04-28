package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class CircularSlewRateLimiter {
  private double positiveRateLimit;
  private double negativeRateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
   * value.
   *
   * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
   *     second. This is expected to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
   *     second. This is expected to be negative.
   * @param initialValue The initial value of the input.
   */
  public CircularSlewRateLimiter(
      double positiveRateLimit, double negativeRateLimit, double initialValue) {
    this.positiveRateLimit = positiveRateLimit;
    this.negativeRateLimit = negativeRateLimit;
    this.prevVal = initialValue;
    this.prevTime = Timer.getFPGATimestamp();
  }

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public CircularSlewRateLimiter(double rateLimit) {
    this(rateLimit, -rateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate. inputs must be angles in radians between [-pi, pi]
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = Timer.getFPGATimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        MathUtil.clamp(
            MathUtil.angleModulus(input - prevVal),
            negativeRateLimit * elapsedTime,
            positiveRateLimit * elapsedTime);
    prevTime = currentTime;
    return prevVal;
  }

  public void setLimits(double neg, double pos) {
    negativeRateLimit = neg;
    positiveRateLimit = pos;
  }

  public void setLimits(double limit) {
    setLimits(-limit, limit);
  }

  /**
   * Returns the value last calculated by the SlewRateLimiter.
   *
   * @return The last value.
   */
  public double lastValue() {
    return prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = Timer.getFPGATimestamp();
  }
}
