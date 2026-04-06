package com.marslib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/**
 * PhotonVision hardware IO implementation for AprilTag tracking.
 *
 * <p>Automatically streams estimated poses and timestamp information into the AdvantageKit log so
 * you can precisely back-calculate vision odometry updates.
 */
public class AprilTagVisionIOPhoton implements AprilTagVisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public AprilTagVisionIOPhoton(String cameraName, PhotonPoseEstimator poseEstimator) {
    this.camera = new PhotonCamera(cameraName);
    this.poseEstimator = poseEstimator;
  }

  @SuppressWarnings("removal")
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isPresent()) {
          EstimatedRobotPose pose = estimatedPose.get();

          inputs.estimatedPoses = new Pose3d[] {pose.estimatedPose};
          inputs.timestamps = new double[] {pose.timestampSeconds};
          inputs.tagCounts = new int[] {result.getTargets().size()};

          double avgDist = 0.0;
          double avgAmbiguity = 0.0;

          for (var target : result.getTargets()) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            avgAmbiguity += target.getPoseAmbiguity();
          }

          inputs.averageDistancesMeters = new double[] {avgDist / result.getTargets().size()};
          inputs.ambiguities = new double[] {avgAmbiguity / result.getTargets().size()};
          return;
        }
      }
    }

    inputs.estimatedPoses = new Pose3d[0];
    inputs.timestamps = new double[0];
    inputs.tagCounts = new int[0];
    inputs.averageDistancesMeters = new double[0];
    inputs.ambiguities = new double[0];
  }
}
