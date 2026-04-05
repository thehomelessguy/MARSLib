package com.marslib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VIOSlamIO {

  @AutoLog
  public static class VIOSlamIOInputs {
    public Pose3d[] estimatedPoses = new Pose3d[0];
    public double[] timestamps = new double[0];
  }

  public default void updateInputs(VIOSlamIOInputs inputs) {}
}
