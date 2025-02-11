package igknighters.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.OptionalInt;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class Triggers {
  private static final AtomicInteger count = new AtomicInteger(0);
  private static final Command counter =
      new Command() {
        // needs access to `this` so can't be done with command factories
        {
          this.schedule();
        }

        public void execute() {
          count.incrementAndGet();
        }
        ;

        public void end(boolean interrupted) {
          this.schedule();
        }
        ;
      }.withName("LoopCounter").ignoringDisable(true);

  static void addCounterToNewScheduler(CommandScheduler scheduler) {
    scheduler.schedule(counter);
  }

  public static int monotonicCycleCounter() {
    return count.get();
  }

  private static EventLoop getDefaultLoop() {
    return CommandScheduler.getInstance().getDefaultButtonLoop();
  }

  public static final class CycleCachedTrigger extends Trigger {
    private final IntSupplier cycleCounter;
    private OptionalInt pollTarget = OptionalInt.empty();

    public CycleCachedTrigger(EventLoop loop, BooleanSupplier trigger, IntSupplier cycleCounter) {
      super(loop, trigger);
      this.cycleCounter = cycleCounter;
    }

    public CycleCachedTrigger(BooleanSupplier trigger, IntSupplier cycleCounter) {
      super(getDefaultLoop(), trigger);
      this.cycleCounter = cycleCounter;
    }

    @Override
    public boolean getAsBoolean() {
      if (super.getAsBoolean()) {
        pollTarget = OptionalInt.of(cycleCounter.getAsInt());
        return true;
      } else if (pollTarget.isPresent() && cycleCounter.getAsInt() == pollTarget.getAsInt()) {
        return true;
      } else {
        pollTarget = OptionalInt.empty();
        return false;
      }
    }
  }

  public static Trigger andAll(Trigger trigger, Trigger... triggers) {
    for (Trigger t : triggers) {
      trigger = trigger.and(t);
    }
    return trigger;
  }

  public static Trigger orAny(Trigger trigger, Trigger... triggers) {
    for (Trigger t : triggers) {
      trigger = trigger.or(t);
    }
    return trigger;
  }

  public static Trigger enterExit(Trigger enter, Trigger exit) {
    Trigger checker =
        new Trigger(
            new BooleanSupplier() {
              boolean output = false;

              @Override
              public boolean getAsBoolean() {
                if (enter.getAsBoolean()) {
                  output = true;
                }
                if (exit.getAsBoolean()) {
                  output = false;
                }
                return output;
              }
            });
    // this is a cheeky way to use the event loop of `enter`
    return enter.or(enter.negate()).and(checker);
  }

  public static Trigger flipFlop(Trigger trigger) {
    return enterExit(trigger, trigger.negate());
  }

  public static Trigger timer(
      EventLoop loop, double targetTime, Timer timer, IntSupplier cycleCounter) {
    // Make the trigger only be high for 1 cycle when the time has elapsed
    return new CycleCachedTrigger(
        loop,
        new BooleanSupplier() {
          double lastTimestamp = -1.0;

          public boolean getAsBoolean() {
            if (!timer.isRunning()) {
              lastTimestamp = -1.0;
              return false;
            }
            double nowTimestamp = timer.get();
            return lastTimestamp < targetTime && nowTimestamp >= targetTime;
          }
        },
        cycleCounter);
  }

  public static Trigger timer(double targetTime, Timer timer) {
    return timer(getDefaultLoop(), targetTime, timer, Triggers::monotonicCycleCounter);
  }

  public static Trigger just(Trigger trigger) {
    Trigger checker =
        new CycleCachedTrigger(
            new BooleanSupplier() {
              boolean last = false;

              @Override
              public boolean getAsBoolean() {
                boolean current = trigger.getAsBoolean();
                boolean output = current && !last;
                last = current;
                return output;
              }
            },
            Triggers::monotonicCycleCounter);
    return trigger.or(trigger.negate()).and(checker);
  }

  /**
   * Trigger that is true when no command is requiring the subsystem or when the default command for
   * the subsystem is running.
   *
   * @param loop the event loop to use
   * @param subsystem the subsystem to check
   * @return the trigger
   */
  public static Trigger subsystemIdle(EventLoop loop, Subsystem subsystem) {
    return new Trigger(
        loop,
        () -> {
          final Command requiring = subsystem.getCurrentCommand();
          return requiring == null || requiring.equals(subsystem.getDefaultCommand());
        });
  }

  /**
   * Trigger that is true when no command is requiring the subsystem or when the default command for
   * the subsystem is running.
   *
   * @param subsystem the subsystem to check
   * @return the trigger
   */
  public static Trigger subsystemIdle(Subsystem subsystem) {
    return subsystemIdle(getDefaultLoop(), subsystem);
  }

  public static Trigger isScheduled(EventLoop loop, Command command) {
    return new Trigger(loop, command::isScheduled);
  }

  public static Trigger isScheduled(Command command) {
    return isScheduled(getDefaultLoop(), command);
  }
}
