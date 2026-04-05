package com.marslib.vision;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

  private final List<VIOSlamIO> slamIOs;
  private final VIOSlamIOInputsAutoLogged[] slamInputs;

  // Base standard deviation multiplier for AprilTags
  private static final double TAG_STD_BASE = 0.05;

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
        if (tagCount == 1 && (ambiguity > 0.2 || Math.abs(pose3d.getZ()) > 0.5)) {
          continue; // Reject noisy single tag or flying robot
        }

        // Exponential scaling based on distance
        // The further away, the exponentially less we trust it
        double linearStdDev = TAG_STD_BASE * Math.exp(avgDist);

        // MegaTag2 Boost
        if (tagCount > 1) {
          linearStdDev *= 0.1; // Extremely tight bound when multi-tag is locked
        }

        double angularStdDev = linearStdDev * 2.0;

        Matrix<N3, N1> stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
        Pose2d pose2d = pose3d.toPose2d();

        swerveDrive.addVisionMeasurement(pose2d, timestamp, stdDevs);
        Logger.recordOutput("Vision/ValidPoses/" + i, pose2d);
      }
    }

    // Process SLAM
    for (int i = 0; i < slamIOs.size(); i++) {
      slamIOs.get(i).updateInputs(slamInputs[i]);
      Logger.processInputs("Vision/SLAM/" + i, slamInputs[i]);

      for (int f = 0; f < slamInputs[i].estimatedPoses.length; f++) {
        Pose3d pose3d = slamInputs[i].estimatedPoses[f];
        double timestamp = slamInputs[i].timestamps[f];

        // Extremely tight static covariance for reliable VIO Odometry
        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.01, 0.01, 0.01);
        Pose2d pose2d = pose3d.toPose2d();

        swerveDrive.addVisionMeasurement(pose2d, timestamp, stdDevs);
        Logger.recordOutput("Vision/SlamPoses/" + i, pose2d);
      }
    }
  }
}
