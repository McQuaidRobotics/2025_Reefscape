package monologue;

import java.lang.invoke.VarHandle;

class Primitives {

  @FunctionalInterface
  public interface DoubleGetter {
    double get(Object o);
  }

  @FunctionalInterface
  public interface LongGetter {
    long get(Object o);
  }

  @FunctionalInterface
  public interface BooleanGetter {
    boolean get(Object o);
  }

  public static final class DoubleVarHandle implements DoubleGetter {
    private final VarHandle handle;

    public DoubleVarHandle(VarHandle handle) {
      this.handle = handle;
    }

    public double get(Object o) {
      return (double) handle.get(o);
    }
  }

  public static final class LongVarHandle implements LongGetter {
    private final VarHandle handle;

    public LongVarHandle(VarHandle handle) {
      this.handle = handle;
    }

    public long get(Object o) {
      return (long) handle.get(o);
    }
  }

  public static final class BooleanVarHandle implements BooleanGetter {
    private final VarHandle handle;

    public BooleanVarHandle(VarHandle handle) {
      this.handle = handle;
    }

    public boolean get(Object o) {
      return (boolean) handle.get(o);
    }
  }
}
