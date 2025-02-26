package igknighters.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.constants.ConstValues.Conv;
import java.util.List;
import monologue.ProceduralStructGenerator;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double FIELD_LENGTH = 690.876 * Conv.INCHES_TO_METERS;
  public static final double FIELD_WIDTH = 317 * Conv.INCHES_TO_METERS;
  public static final double STARTING_LINE_X = inchesToMeters(298.438);

  public static class Processor {
    public static final Pose2d CENTER_FACE =
        new Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d FAR_CAGE =
        new Translation2d(inchesToMeters(345.428), inchesToMeters(286.779));
    public static final Translation2d MIDDLE_CAGE =
        new Translation2d(inchesToMeters(345.428), inchesToMeters(242.855));
    public static final Translation2d CLOSE_CAGE =
        new Translation2d(inchesToMeters(345.428), inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double DEEP_CAGE_HEIGHT = inchesToMeters(3.125);
    public static final double SHALLOW_CAGE_HEIGHT = inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d LEFT_CENTER_FACE =
        new Pose2d(
            inchesToMeters(33.526), inchesToMeters(291.176), Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d RIGHT_CENTER_FACE =
        new Pose2d(
            inchesToMeters(33.526), inchesToMeters(25.824), Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class AutoGamePieces {
    // Measured from the center of the ice cream
    public static final Pose2d LEFT_GAMEPIECE_STACK =
        new Pose2d(inchesToMeters(48), inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d MIDDLE_GAMEPIECE_STACK =
        new Pose2d(inchesToMeters(48), inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d RIGHT_GAMEPIECE_STACK =
        new Pose2d(inchesToMeters(48), inchesToMeters(86.5), new Rotation2d());
  }

  public static final class Reef {
    public static final Translation2d CENTER =
        new Translation2d(inchesToMeters(176.746), inchesToMeters(158.501));
    private static final Pose2d CENTER_POSE = new Pose2d(CENTER, Rotation2d.kZero);
    private static final Translation2d FACE_OFFSET = new Translation2d(inchesToMeters(32.75), 0.0);

    public enum BranchHeight implements StructSerializable {
      L4(inchesToMeters(72), -90 * Conv.DEGREES_TO_RADIANS),
      L3(inchesToMeters(47.625), -35 * Conv.DEGREES_TO_RADIANS),
      L2(inchesToMeters(31.875), -35 * Conv.DEGREES_TO_RADIANS),
      L1(inchesToMeters(18), 0);

      BranchHeight(double height, double pitch) {
        this.height = height;
        this.pitch = pitch;
      }

      public final double height;
      public final double pitch;

      public static final Struct<BranchHeight> struct =
          ProceduralStructGenerator.genEnum(BranchHeight.class);
    }

    public enum Side implements StructSerializable {
      CLOSE_LEFT(Rotation2d.fromDegrees(120.0)),
      CLOSE_MID(Rotation2d.fromDegrees(180.0)),
      CLOSE_RIGHT(Rotation2d.fromDegrees(-120.0)),
      FAR_LEFT(Rotation2d.fromDegrees(60.0)),
      FAR_MID(Rotation2d.fromDegrees(0.0)),
      FAR_RIGHT(Rotation2d.fromDegrees(-60.0));

      /**
       * The position of the center of the face of the reef pointing away from the center of the
       * reef.
       */
      public final Pose2d face;

      private Side(Rotation2d angle) {
        this.face = Reef.CENTER_POSE.plus(new Transform2d(FACE_OFFSET.rotateBy(angle), angle));
      }

      public static final Pose2d[] FACES = {
        CLOSE_LEFT.face,
        CLOSE_MID.face,
        CLOSE_RIGHT.face,
        FAR_LEFT.face,
        FAR_MID.face,
        FAR_RIGHT.face
      };

      private static final double BRANCH_OFFSET = inchesToMeters(6.5);

      private Pose2d scorePose(double distFromFace, double yOffset) {
        Translation2d t =
            FAR_MID
                .face
                .getTranslation()
                .plus(new Translation2d(distFromFace, yOffset))
                .rotateAround(CENTER, this.face.getRotation());
        return new Pose2d(t, this.face.getRotation().rotateBy(Rotation2d.kPi));
      }

      public Pose2d alignScoreLeft(double distFromFace) {
        return scorePose(distFromFace, -BRANCH_OFFSET);
      }

      public Pose2d alignScoreRight(double distFromFace) {
        return scorePose(distFromFace, BRANCH_OFFSET);
      }

      public Pose2d alignScoreCenter(double distFromFace) {
        return scorePose(distFromFace, 0.0);
      }

      public static final Struct<Side> struct = ProceduralStructGenerator.genEnum(Side.class);
    }
  }

  public static final AprilTagFieldLayout APRIL_TAG_FIELD =
      new AprilTagFieldLayout(
          List.of(AprilTags.TAGS), FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

  public static final Translation2d TRANSLATION2D_CENTER =
      new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, FieldConstants.FIELD_WIDTH / 2.0);
  public static final Pose2d POSE2D_CENTER = new Pose2d(TRANSLATION2D_CENTER, Rotation2d.kZero);
}
