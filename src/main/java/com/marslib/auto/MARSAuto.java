package com.marslib.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Utility Command factory for autonomous routines.
 *
 * <p>Students: This class hooks directly into PathPlanner and Choreo. Use `runChoreoTrajectory` to
 * execute the exact pre-rendered JSON files mapped in your deploy directory natively.
 */
public class MARSAuto {

  /**
   * Executes a pre-planned, time-optimal Choreo trajectory via PathPlannerLib. Retains support for
   * PathPlanner NamedCommands hooks.
   *
   * @param choreoTrajectoryName The filename of the trajectory inside deploy/choreo
   * @return The autonomous execution Command
   */
  public static Command runChoreoTrajectory(String choreoTrajectoryName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreoTrajectoryName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.err.println("Failed to load Choreo trajectory: " + choreoTrajectoryName);
      e.printStackTrace();
      return new edu.wpi.first.wpilibj2.command.PrintCommand("Fallback: Trajectory failed to load");
    }
  }

  /**
   * Utilizes PathPlanner's A* Pathfinding to navigate avoiding obstacles. Helpful for dynamic
   * teleop routing.
   *
   * @param targetPose The destination field pose
   * @return Pathfinding command
   */
  public static Command pathfindObstacleAvoidance(Pose2d targetPose) {
    // Tunable dynamic pathfinding constraints
    PathConstraints constraints =
        new PathConstraints(
            3.0, // Max Velocity (m/s)
            2.0, // Max Acceleration (m/s^2)
            Math.PI, // Max Angular Velocity (rad/s)
            Math.PI / 2 // Max Angular Acceleration (rad/s^2)
            );

    // Pathfind dynamically to target, ending with 0 velocity
    return AutoBuilder.pathfindToPose(
        targetPose, constraints, 0.0 // Goal end velocity
        );
  }
}
