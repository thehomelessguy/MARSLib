package com.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {
  public static class PoseEstimate {
    public Pose3d pose = new Pose3d();
    public double timestampSeconds = 0.0;
    public double latency = 0.0;
    public int tagCount = 0;
    public double tagSpan = 0.0;
    public double avgTagDist = 0.0;
    public double avgTagArea = 0.0;
    public edu.wpi.first.math.geometry.Pose3d[] rawFiducials = new Pose3d[0];
    public double rawPitch = 0.0;
    public double rawYaw = 0.0;
    public double rawArea = 0.0;
    public boolean isBotPose = false;
  }

  public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
    return new PoseEstimate();
  }

  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
    return new PoseEstimate();
  }

  /**
   * Pushes the robot's orientation directly to the Limelight over NetworkTables. This is required
   * for accurate Megatag 2 solve generation and pitch correction.
   */
  public static void SetRobotOrientation(
      String limelightName,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    double[] entries = new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate};
    NetworkTableInstance.getDefault()
        .getTable(limelightName)
        .getEntry("robot_orientation_set")
        .setDoubleArray(entries);
  }
}
