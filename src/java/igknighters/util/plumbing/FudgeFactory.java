package igknighters.util.plumbing;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class FudgeFactory {
  private static final String pathPrefix = "/Tunables/Fudge/";

  public static abstract class Fudge<T> {
    protected final String path;
    protected T memory;

    protected Fudge(String path, T value) {
      this.path = path;
      this.memory = value;
    }

    public T fetch() {
      return memory;
    }

    public void store(T value) {
      memory = value;
    }

    public void flash(T value) {
      memory = value;
      flush();
    }

    public abstract void flush();

    public abstract void fromDisk();
  }

  private static class DoubleFudge extends Fudge<Double> {
    private final DoubleEntry entry;

    private DoubleFudge(String name, Double value) {
      super(name, value);
      var topic = NetworkTableInstance.getDefault()
          .getDoubleTopic(pathPrefix + name);
      if (topic.isPersistent()) {
        entry = topic.getEntry(value);
      } else {
        entry = topic.getEntry(value);
        entry.setDefault(value);
        topic.setPersistent(true);
      }
      memory = entry.get();
    }

    @Override
    public void flush() {
      entry.set(memory);
    }

    @Override
    public void fromDisk() {
      memory = entry.get();
    }
  }

  private static class StructFudge<T, S extends Struct<T>> extends Fudge<T> {
    private final StructEntry<T> entry;

    private StructFudge(String name, T value, S struct) {
      super(name, value);
      var topic = NetworkTableInstance.getDefault()
          .getStructTopic(pathPrefix + name, struct);
      if (topic.isPersistent()) {
        entry = topic.getEntry(value);
      } else {
        entry = topic.getEntry(value);
        entry.setDefault(value);
        topic.setPersistent(true);
      }
      memory = entry.get();
    }

    @Override
    public void flush() {
      entry.set(memory);
    }

    @Override
    public void fromDisk() {
      memory = entry.get();
    }
  }

  public static Fudge<Double> fudge(String name, double value) {
    return new DoubleFudge(name, value);
  }

  public static <T, S extends Struct<T> & StructSerializable> Fudge<T> fudge(String name, T value, S struct) {
    return new StructFudge<>(name, value, struct);
  }
}
