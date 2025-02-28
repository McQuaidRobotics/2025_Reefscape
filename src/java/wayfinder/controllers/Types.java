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
          Math.min(maxVelocity, other.maxVelocity), Math.min(maxAcceleration, other.maxAcceleration));
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
}
