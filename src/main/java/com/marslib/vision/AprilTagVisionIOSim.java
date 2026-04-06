package com.marslib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
  private static VisionSystemSim visionSim;
  private static AprilTagFieldLayout fieldLayout;

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator poseEstimator;
  private final Supplier<Pose2d> poseSupplier;

  public AprilTagVisionIOSim(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    // Initialize global vision sim once
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      try {
        fieldLayout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
      } catch (Exception e) {
        fieldLayout = new AprilTagFieldLayout(new java.util.ArrayList<>(), 16.54, 8.21);
      }
      visionSim.addAprilTags(fieldLayout);
    }

    camera = new PhotonCamera(cameraName);

    // Setup camera sim properties
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.25, 0.08); // Simulate realistic calibration errors
    cameraProp.setFPS(20); // 20 FPS simulated
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(15);

    cameraSim = new PhotonCameraSim(camera, cameraProp);

    visionSim.addCamera(cameraSim, robotToCamera);

    @SuppressWarnings("removal")
    PhotonPoseEstimator tmpEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator = tmpEstimator;
  }

  @SuppressWarnings("removal")
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    // Update simulation view from current true pose so cameras can 'see' the field
    visionSim.update(poseSupplier.get());

    var results = camera.getAllUnreadResults();

    boolean simulatedOcclusion = false;
    if (frc.robot.Constants.SimulationConstants.ENABLE_VISION_OCCLUSION) {
      simulatedOcclusion =
          Math.random() < frc.robot.Constants.SimulationConstants.VISION_OCCLUSION_DROP_PROBABILITY;
    }

    if (!results.isEmpty() && !simulatedOcclusion) {
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
