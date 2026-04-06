package com.marslib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.Test;

public class TagExtractor {
  @Test
  public void printTags() {
    try {
      AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      System.out.println("2026 REBUILT TAG LAYOUT:");
      // Tag ID 1 through 22
      for (int i = 1; i <= 22; i++) {
        if (layout.getTagPose(i).isPresent()) {
          Pose3d pose = layout.getTagPose(i).get();
          System.out.printf(
              "Tag %d: X: %.2f, Y: %.2f, Z: %.2f\n", i, pose.getX(), pose.getY(), pose.getZ());
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
