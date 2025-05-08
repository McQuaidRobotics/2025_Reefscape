package monologue;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.struct.Struct;
import java.lang.ref.WeakReference;
import java.util.Optional;

public class MonologueBackend implements EpilogueBackend {
  sealed interface Location {
    record Path(String path) implements Location {
      public Path(String path) {
        this.path = NetworkTable.normalizeKey(path, true);
      }
    }

    record Object(WeakReference<Logged> weakObject, Optional<String> prefix) implements Location {
      public Optional<String> extendedPrefix(String path) {
        if (path == null || path.isEmpty()) {
          return prefix;
        } else if (prefix.isPresent()) {
          return Optional.of(prefix.get() + "/" + path);
        } else {
          return Optional.of(path);
        }
      }

      public String getFullPath(String key) {
        if (prefix.isPresent()) {
          return prefix.get() + "/" + key;
        } else {
          return key;
        }
      }

      public Logged object() {
        return weakObject.get();
      }
    }
  }

  private final Location location;
  private final LogSink sink;

  MonologueBackend(Location location, LogSink sink) {
    this.location = location;
    this.sink = sink;
  }

  MonologueBackend(String path, LogSink sink) {
    this.location = new Location.Path(path);
    this.sink = sink;
  }

  MonologueBackend(Logged object, String subTable, LogSink sink) {
    this.location = new Location.Object(new WeakReference<>(object), Optional.ofNullable(subTable));
    this.sink = sink;
  }

  MonologueBackend(Logged object, LogSink sink) {
    this.location = new Location.Object(new WeakReference<>(object), Optional.empty());
    this.sink = sink;
  }

  public LogSink getSink() {
    return sink;
  }

  public boolean stillValid() {
    if (location instanceof Location.Path) {
      return true;
    } else if (location instanceof Location.Object objectLoc) {
      return !objectLoc.weakObject().refersTo(null)
          && !Logged.getNodes(objectLoc.object()).isEmpty();
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public MonologueBackend getNested(String path) {
    path = NetworkTable.normalizeKey(path, false);
    if (location instanceof Location.Path pathLoc) {
      return new MonologueBackend(pathLoc.path() + "/" + path, sink);
    } else if (location instanceof Location.Object objectLoc) {
      return new MonologueBackend(
          new Location.Object(objectLoc.weakObject(), objectLoc.extendedPrefix(path)), sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, boolean value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, double value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, String value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public <S> void log(String identifier, S value, Struct<S> struct) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, struct, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, struct, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, float value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, long value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, int value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, boolean[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, double[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, String[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, float[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, long[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, int[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public <S> void log(String identifier, S[] value, Struct<S> struct) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, struct, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, struct, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }

  @Override
  public void log(String identifier, byte[] value) {
    if (!stillValid()) return;
    identifier = NetworkTable.normalizeKey(identifier, false);
    if (location instanceof Location.Path pathLoc) {
      Monologue.log(pathLoc.path() + "/" + identifier, value, sink);
    } else if (location instanceof Location.Object objectLoc) {
      String fullPath = objectLoc.getFullPath(identifier);
      objectLoc.object().log(fullPath, value, sink);
    } else {
      throw new IllegalStateException("Unknown location type: " + location.getClass());
    }
  }
}
