// Copyright (c) Choreo contributors

package choreo.trajectory;

import choreo.util.ChoreoAllianceFlipUtil;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * The generic interface for a sample in a trajectory.
 *
 * @param <Self> Derived sample type.
 */
public interface TrajectorySample<Self extends TrajectorySample<Self>>
    extends Interpolatable<Self>, StructSerializable {
  /**
   * Returns the timestamp of this sample.
   *
   * @return the timestamp of this sample.
   */
  double getTimestamp();

  /**
   * Returns the pose at this sample.
   *
   * @return the pose at this sample.
   */
  Pose2d getPose();

  /**
   * Returns the field-relative chassis speeds of this sample.
   *
   * @return the field-relative chassis speeds of this sample.
   */
  ChassisSpeeds getChassisSpeeds();

  /**
   * Returns this sample, mirrored across the field midline.
   *
   * @param flipper the flipper to use.
   * @return this sample, mirrored across the field midline.
   */
  Self flipped(Flipper flipper);

  /**
   * Returns this sample, mirrored across the field midline.
   *
   * @return this sample, mirrored across the field midline.
   */
  default Self flipped() {
    return flipped(ChoreoAllianceFlipUtil.getFlipper());
  }

  /**
   * Returns this sample, offset by the given timestamp.
   *
   * @param timestampOffset the offset to apply to the timestamp.
   * @return this sample, offset by the given timestamp.
   */
  Self offsetBy(double timestampOffset);
}
