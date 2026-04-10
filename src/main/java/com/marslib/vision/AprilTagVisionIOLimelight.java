package com.marslib.vision;

import com.limelight.LimelightHelpers;
import com.limelight.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Real hardware implementation of {@link AprilTagVisionIO} using a Limelight camera.
 *
 * <p>Fetches MegaTag2 pose estimates via the {@link LimelightHelpers} JSON API. Ambiguity is
 * defaulted to 0.0 because Limelight does not expose per-target pose ambiguity natively; downstream
 * quality control relies on distance-based standard deviation scaling in {@link MARSVision}.
 */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  private final String cameraName;

  public AprilTagVisionIOLimelight(String cameraName) {
    this.cameraName = cameraName;
  }

  @Override
  public void setRobotOrientation(
      double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        cameraName,
        Math.toDegrees(yaw),
        Math.toDegrees(yawRate),
        Math.toDegrees(pitch),
        Math.toDegrees(pitchRate),
        Math.toDegrees(roll),
        Math.toDegrees(rollRate));
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    // Utilize traditional MegaTag2 / BotPose wpiBlue_MegaTag2
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if (estimate != null && estimate.tagCount > 0) {
      inputs.estimatedPoses = new Pose3d[] {estimate.pose};
      // Standardize timestamp exactly as WPILib
      inputs.timestamps = new double[] {estimate.timestampSeconds};
      inputs.tagCounts = new int[] {estimate.tagCount};
      inputs.averageDistancesMeters = new double[] {estimate.avgTagDist};

      // Limelight MegaTag2 does not expose per-tag pose ambiguity natively.
      // Default to 0.0 to avoid tripping the MAX_AMBIGUITY filter in MARSVision.
      // Quality control for single-tag observations is handled by downstream
      // distance-based standard deviation scaling instead.
      inputs.ambiguities = new double[] {0.0};
    } else {
      inputs.estimatedPoses = new Pose3d[0];
      inputs.timestamps = new double[0];
      inputs.tagCounts = new int[0];
      inputs.averageDistancesMeters = new double[0];
      inputs.ambiguities = new double[0];
    }
  }
}
