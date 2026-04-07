package com.marslib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import gg.questnav.QuestNav;

/**
 * Real hardware implementation of {@link VIOSlamIO} using a Meta Quest headset running QuestNav.
 *
 * <p>Provides 6-DOF inside-out visual-inertial odometry (VIO) pose estimates via the QuestNav
 * NetworkTables bridge. Latency is compensated by subtracting the reported network delay from the
 * current FPGA timestamp.
 */
public class VIOSlamIOQuestNav implements VIOSlamIO {

  private final QuestNav questNav;

  public VIOSlamIOQuestNav() {
    this.questNav = new QuestNav();
  }

  @Override
  public void updateInputs(VIOSlamIOInputs inputs) {
    if (questNav.isConnected()) {
      Pose3d pose = questNav.getPose3d();
      // Calculate Network latency using NT4 delay or provided timing API
      double timestamp = Timer.getFPGATimestamp() - questNav.getLatencySeconds();

      inputs.estimatedPoses = new Pose3d[] {pose};
      inputs.timestamps = new double[] {timestamp};
    } else {
      inputs.estimatedPoses = new Pose3d[0];
      inputs.timestamps = new double[0];
    }
  }
}
