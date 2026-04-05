package com.marslib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {

  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] estimatedPoses = new Pose3d[0];
    public double[] timestamps = new double[0];
    public int[] tagCounts = new int[0];
    public double[] averageDistancesMeters = new double[0];
    public double[] ambiguities = new double[0];
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
