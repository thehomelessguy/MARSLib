package com.marslib.vision;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Central aggregator for all visual and SLAM external odometry integrations.
 *
 * <p>Students: This subsystem actively pools array data from unlimited AprilTag and VIO SLAM
 * pipelines. It analyzes the physical ambiguity of every single target, scales dynamic standard
 * deviations exponentially based on target distance, and mathematically rejects 'hallucinatory'
 * camera frames before passing the secure coordinates into the SwerveDrive Pose Estimator.
 */
public class MARSVision extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final List<AprilTagVisionIO> aprilTagIOs;
  private final AprilTagVisionIOInputsAutoLogged[] aprilTagInputs;
  private java.util.Optional<Translation2d> latestTargetTranslation = java.util.Optional.empty();

  private final List<VIOSlamIO> slamIOs;
  private final VIOSlamIOInputsAutoLogged[] slamInputs;

  /**
   * Constructs the absolute Vision mapping structure.
   *
   * @param swerveDrive The primary SwerveDrive subsystem reference for data injection.
   * @param aprilTagIOs A list of all active AprilTag IO architectures (Limelight, Photon).
   * @param slamIOs A list of all active VIO SLAM IO architectures (QuestNav, ROS2).
   */
  public MARSVision(
      SwerveDrive swerveDrive, List<AprilTagVisionIO> aprilTagIOs, List<VIOSlamIO> slamIOs) {
    this.swerveDrive = swerveDrive;
    this.aprilTagIOs = aprilTagIOs;
    this.aprilTagInputs = new AprilTagVisionIOInputsAutoLogged[aprilTagIOs.size()];
    for (int i = 0; i < aprilTagInputs.length; i++) {
      aprilTagInputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }

    this.slamIOs = slamIOs;
    this.slamInputs = new VIOSlamIOInputsAutoLogged[slamIOs.size()];
    for (int i = 0; i < slamInputs.length; i++) {
      slamInputs[i] = new VIOSlamIOInputsAutoLogged();
    }
  }

  /**
   * The active periodic layer. Scans through all mapped interfaces exactly once per loop. Executes
   * bounding box Z-height rejection and ambiguity thresholding natively.
   */
  @Override
  public void periodic() {
    latestTargetTranslation =
        java.util.Optional.empty(); // Reset each loop unless a target is found

    // Process AprilTags
    for (int i = 0; i < aprilTagIOs.size(); i++) {
      aprilTagIOs.get(i).updateInputs(aprilTagInputs[i]);
      Logger.processInputs("Vision/AprilTag/" + i, aprilTagInputs[i]);

      for (int f = 0; f < aprilTagInputs[i].estimatedPoses.length; f++) {
        Pose3d pose3d = aprilTagInputs[i].estimatedPoses[f];
        int tagCount = aprilTagInputs[i].tagCounts[f];
        double ambiguity = aprilTagInputs[i].ambiguities[f];
        double avgDist = aprilTagInputs[i].averageDistancesMeters[f];
        double timestamp = aprilTagInputs[i].timestamps[f];

        // Dynamic Filtering & Ambiguity Rejection
        if (tagCount == 1
            && (ambiguity > frc.robot.constants.VisionConstants.MAX_AMBIGUITY.get()
                || Math.abs(pose3d.getZ())
                    > frc.robot.constants.VisionConstants.MAX_Z_HEIGHT.get())) {
          continue; // Reject noisy single tag or flying robot
        }

        // Quadratic scaling based on distance
        // The further away, the exponentially less we trust it (squared)
        double linearStdDev =
            frc.robot.constants.VisionConstants.TAG_STD_BASE.get() * Math.pow(avgDist, 2);

        // MegaTag2 Boost: Dramatically tighten bounds when multiple tags are visible
        if (tagCount > 1) {
          linearStdDev *= frc.robot.constants.VisionConstants.MULTI_TAG_STD_MULTIPLIER.get();
        }

        double angularStdDev =
            linearStdDev * frc.robot.constants.VisionConstants.ANGULAR_STD_MULTIPLIER.get();

        Matrix<N3, N1> stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
        Pose2d pose2d = pose3d.toPose2d();

        swerveDrive.addVisionMeasurement(pose2d, timestamp, stdDevs);
        Logger.recordOutput("Vision/ValidPoses/" + i, pose2d);
        latestTargetTranslation = java.util.Optional.of(pose2d.getTranslation());
      }
    }

    // Process SLAM
    for (int i = 0; i < slamIOs.size(); i++) {
      slamIOs.get(i).updateInputs(slamInputs[i]);
      Logger.processInputs("Vision/SLAM/" + i, slamInputs[i]);

      for (int f = 0; f < slamInputs[i].estimatedPoses.length; f++) {
        Pose3d pose3d = slamInputs[i].estimatedPoses[f];
        double timestamp = slamInputs[i].timestamps[f];

        // Tight static covariance for reliable VIO odometry
        Matrix<N3, N1> stdDevs =
            VecBuilder.fill(
                frc.robot.constants.VisionConstants.SLAM_STD_DEV.get(),
                frc.robot.constants.VisionConstants.SLAM_STD_DEV.get(),
                frc.robot.constants.VisionConstants.SLAM_ANGULAR_STD_DEV.get());
        Pose2d pose2d = pose3d.toPose2d();

        swerveDrive.addVisionMeasurement(pose2d, timestamp, stdDevs);
        Logger.recordOutput("Vision/SlamPoses/" + i, pose2d);
      }
    }
  }

  /**
   * Retrieves the latest, rigorously vetted vision-based target translation. This provides a direct
   * fallback for SOTM and aiming loops if odometry is drifting.
   */
  public java.util.Optional<Translation2d> getBestTargetTranslation() {
    return latestTargetTranslation;
  }
}
