package com.marslib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.geometry.Ray;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.DetectFilter;
import org.dyn4j.world.World;
import org.dyn4j.world.result.RaycastResult;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates a 360-degree LiDAR or Time-of-Flight sensor array using dyn4j raycasting. Shoots rays
 * outward from the robot chassis to detect walls and game pieces.
 */
public class LidarIOSim {
  private static final int NUM_RAYS = 16;
  private static final double MAX_RAY_DISTANCE_METERS = 3.0; // Max Range

  /**
   * Performs the raycast sweep around the robot and logs the hits as a 3D Point Cloud.
   *
   * @param robotPose The current field-relative pose of the robot.
   */
  public void updateInputs(Pose2d robotPose) {
    World<org.dyn4j.dynamics.Body> world = MARSPhysicsWorld.getInstance().getDyn4jWorld();

    // Safety check if the user hasn't successfully initialized a world.
    if (world == null) return;

    Vector2 origin = new Vector2(robotPose.getX(), robotPose.getY());
    List<Translation3d> pointCloud = new ArrayList<>();

    // We only want to raycast against obstacles/game pieces, not the chassis itself
    // Filtering mechanism: we will just skip intersections if distance is precisely 0
    // (which usually happens if querying from inside the chassis body).

    for (int i = 0; i < NUM_RAYS; i++) {
      double angleRad = robotPose.getRotation().getRadians() + (i * ((2 * Math.PI) / NUM_RAYS));
      Vector2 direction = new Vector2(Math.cos(angleRad), Math.sin(angleRad));

      Ray ray = new Ray(origin, direction);

      // Perform broad-phase raycast finding the closest object
      RaycastResult<org.dyn4j.dynamics.Body, org.dyn4j.dynamics.BodyFixture> result =
          world.raycastClosest(ray, MAX_RAY_DISTANCE_METERS, new DetectFilter<>(true, true, null));

      if (result != null && result.getRaycast().getDistance() > 0.01) {
        // We hit something! Convert the distance back to an absolute field Point3d
        double distance = result.getRaycast().getDistance();
        double hitX = origin.x + (direction.x * distance);
        double hitY = origin.y + (direction.y * distance);

        pointCloud.add(new Translation3d(hitX, hitY, 0.2)); // Render hits at standard bumper height
      }
    }

    // Convert list to native array
    Translation3d[] pointCloudArray = new Translation3d[pointCloud.size()];
    pointCloud.toArray(pointCloudArray);

    // Log directly to AdvantageKit for 3D View rendering
    Logger.recordOutput("PhysicsSim/LidarPointCloud", pointCloudArray);
  }
}
