package com.marslib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Frustum Visualizer for AdvantageScope 3D telemetry.
 *
 * <p>Intercepts the physical limits (FOV) of simulated cameras (e.g., Limelight) and structurally
 * casts their bounding boxes into a WPILib Pose3d array. This array is natively logged using
 * AdvantageKit and renders semi-transparent lines in the simulator showing mathematically exactly
 * where the camera can see.
 */
public class FrustumVisualizer {

  /**
   * Generate an array of 5 3D poses representing a visual cone/frustum.
   *
   * @param cameraPose The exact physical mounting pose of the camera on the field (Robot Pose +
   *     Camera Transform).
   * @param horizontalFOV The horizontal FOV in degrees (e.g. Limelight 3 = 63.3)
   * @param verticalFOV The vertical FOV in degrees (e.g. Limelight 3 = 49.7)
   * @param clipDistance How far out to draw the bounding rays in meters.
   * @return Pose3d array for AdvantageScope 3D rendering.
   */
  public static Pose3d[] generateFrustum(
      Pose3d cameraPose, double horizontalFOV, double verticalFOV, double clipDistance) {

    double hFovRads = Math.toRadians(horizontalFOV / 2.0);
    double vFovRads = Math.toRadians(verticalFOV / 2.0);

    // Vector translations relative to the camera origin extending outward to the clip plane.
    Translation3d topLeft =
        new Translation3d(
            clipDistance, clipDistance * Math.tan(hFovRads), clipDistance * Math.tan(vFovRads));
    Translation3d topRight =
        new Translation3d(
            clipDistance, -clipDistance * Math.tan(hFovRads), clipDistance * Math.tan(vFovRads));
    Translation3d bottomLeft =
        new Translation3d(
            clipDistance, clipDistance * Math.tan(hFovRads), -clipDistance * Math.tan(vFovRads));
    Translation3d bottomRight =
        new Translation3d(
            clipDistance, -clipDistance * Math.tan(hFovRads), -clipDistance * Math.tan(vFovRads));

    Pose3d p1 = cameraPose.plus(new Transform3d(topLeft, new Rotation3d()));
    Pose3d p2 = cameraPose.plus(new Transform3d(topRight, new Rotation3d()));
    Pose3d p3 = cameraPose.plus(new Transform3d(bottomLeft, new Rotation3d()));
    Pose3d p4 = cameraPose.plus(new Transform3d(bottomRight, new Rotation3d()));

    // The exact origin of the lens
    Pose3d p0 = cameraPose;

    // Structured sequentially so AdvantageScope 'Component' drawer draws connected lines.
    return new Pose3d[] {p0, p1, p2, p0, p3, p4, p0, p1, p3, p2, p4};
  }
}
