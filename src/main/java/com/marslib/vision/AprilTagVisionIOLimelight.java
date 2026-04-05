package com.marslib.vision;

import com.limelight.LimelightHelpers;
import com.limelight.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  private final String cameraName;

  public AprilTagVisionIOLimelight(String cameraName) {
    this.cameraName = cameraName;
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    // Utilize traditional MegaTag2 / BotPose wpiBlue
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (estimate != null && estimate.tagCount > 0) {
      inputs.estimatedPoses = new Pose3d[] {estimate.pose};
      // Standardize timestamp exactly as WPILib
      inputs.timestamps = new double[] {estimate.timestampSeconds};
      inputs.tagCounts = new int[] {estimate.tagCount};
      inputs.averageDistancesMeters = new double[] {estimate.avgTagDist};

      // Limelight 3G / later may populate ambiguity, otherwise default to 0.0 for multitag
      inputs.ambiguities =
          new double[] {estimate.tagCount > 1 ? 0.0 : 0.9}; // arbitrary fallback if not parsed
    } else {
      inputs.estimatedPoses = new Pose3d[0];
      inputs.timestamps = new double[0];
      inputs.tagCounts = new int[0];
      inputs.averageDistancesMeters = new double[0];
      inputs.ambiguities = new double[0];
    }
  }
}
