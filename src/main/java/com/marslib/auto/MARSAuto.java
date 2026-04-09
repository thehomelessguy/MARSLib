package com.marslib.auto;

import static frc.robot.constants.ModeConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.*;
import frc.robot.constants.AutoConstants;

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
      DriverStation.reportError(
          "Failed to load Choreo trajectory: " + choreoTrajectoryName + "\n" + e.getMessage(),
          e.getStackTrace());
      return new edu.wpi.first.wpilibj2.command.PrintCommand("Fallback: Trajectory failed to load");
    }
  }

  /**
   * Executes a pre-planned Choreo trajectory but dynamically pathfinds to the start position first
   * if the robot is off-target. This is the ultimate "Hybrid" approach.
   *
   * @param choreoTrajectoryName The filename of the trajectory inside deploy/choreo
   * @return The autonomous execution Command
   */
  public static Command pathfindThenRunChoreoTrajectory(String choreoTrajectoryName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreoTrajectoryName);

      // Constraints for navigating to the starting pose of the trajectory
      PathConstraints pathfindingConstraints =
          new PathConstraints(
              AutoConstants.MAX_VELOCITY_MPS,
              AutoConstants.MAX_ACCELERATION_MPS2,
              AutoConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
              AutoConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2);

      return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load Choreo trajectory for pathfinding: "
              + choreoTrajectoryName
              + "\n"
              + e.getMessage(),
          e.getStackTrace());
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
            AutoConstants.MAX_VELOCITY_MPS,
            AutoConstants.MAX_ACCELERATION_MPS2,
            AutoConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2);

    // Pathfind dynamically to target, ending with 0 velocity
    return AutoBuilder.pathfindToPose(
        targetPose, constraints, 0.0 // Goal end velocity
        );
  }
}
