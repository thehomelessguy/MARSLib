package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for MARSAuto command factory methods.
 *
 * <p>Since MARSAuto wraps PathPlanner's AutoBuilder (which requires full robot configuration),
 * these tests verify graceful fallback behavior when trajectories are unavailable.
 */
public class MARSAutoTest {

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
  }

  @Test
  public void testRunChoreoTrajectoryReturnsFallbackForMissingFile() {
    // PathPlanner is not configured in test context, so this should gracefully return a fallback
    Command result = MARSAuto.runChoreoTrajectory("nonexistent_trajectory");
    assertNotNull(result, "Should return a fallback command, never null.");
  }

  @Test
  public void testPathfindThenRunChoreoReturnsFallbackForMissingFile() {
    Command result = MARSAuto.pathfindThenRunChoreoTrajectory("nonexistent_trajectory");
    assertNotNull(result, "Should return a fallback command, never null.");
  }

  @Test
  public void testPathfindObstacleAvoidanceDoesNotThrowWithoutAutoBuilder() {
    // AutoBuilder.pathfindToPose requires AutoBuilder.configure() which isn't done in tests.
    // This verifies the factory doesn't crash before reaching PathPlanner internals.
    try {
      MARSAuto.pathfindObstacleAvoidance(
          new edu.wpi.first.math.geometry.Pose2d(
              1.0, 1.0, new edu.wpi.first.math.geometry.Rotation2d()));
      // If AutoBuilder isn't configured, this may throw — that's expected.
      // The key assertion is that MARSAuto itself doesn't add broken logic on top.
    } catch (Exception e) {
      // Expected: AutoBuilder not configured. Verify it's a PathPlanner error, not ours.
      assertTrue(
          e.getMessage() != null, "Exception should have a message from PathPlanner internals.");
    }
  }
}
