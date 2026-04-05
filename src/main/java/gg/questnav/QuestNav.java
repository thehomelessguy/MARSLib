package gg.questnav;

import edu.wpi.first.math.geometry.Pose3d;

public class QuestNav {
  public boolean isConnected() {
    return false;
  }

  public Pose3d getPose3d() {
    return new Pose3d();
  }

  public double getLatencySeconds() {
    return 0.0;
  }
}
