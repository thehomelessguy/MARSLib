package com.marslib.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marslib.mechanisms.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import java.util.List;
import org.junit.jupiter.api.Test;

public class MARSPhysicsWorldTest {

  @org.junit.jupiter.api.BeforeEach
  @org.junit.jupiter.api.AfterEach
  public void wipePhysicsWorld() {
    com.marslib.testing.MARSTestHarness.reset();
  }

  @Test
  public void testVoltageSagFloor() {
    MARSPhysicsWorld world = MARSPhysicsWorld.getInstance();

    // Inject massive current block to emulate sudden stall or short
    world.addFrameCurrentDrawAmps(800.0);
    world.update(0.02);

    // The loaded voltage calculation should sag but bottom out at 6.0V
    assertEquals(
        6.0,
        world.getSimulatedVoltage(),
        0.01,
        "Voltage sag must be floored at 6.0V to prevent mathematical death spirals");
  }

  @Test
  public void testGamePieceInitialSpawning() {
    MARSPhysicsWorld world = MARSPhysicsWorld.getInstance();
    List<GamePieceSim> pieces = world.getGamePieces();

    // Should spawn the 4 middle pieces initially
    assertTrue(pieces.size() >= 4, "Physics world should instantiate 4 initial game pieces");
  }

  @Test
  public void testIntakeRadiusEvaluation() {
    MARSPhysicsWorld world = MARSPhysicsWorld.getInstance();
    List<GamePieceSim> pieces = world.getGamePieces();

    GamePieceSim targetPiece = null;
    for (GamePieceSim p : pieces) {
      if (!p.isIntaked()) {
        targetPiece = p;
        break;
      }
    }

    assertNotNull(
        targetPiece, "There should be at least one active game piece available to swallow");

    // Spoof robot pose exactly on top of the game piece to enforce a collision trigger
    Pose2d spoofedRobotPose = new Pose2d(targetPiece.getPosition(), new Rotation2d());

    int gotPiece =
        world.checkIntake(spoofedRobotPose, 0.5, 40); // 0.5m radius collection zone, max 40 pieces

    assertTrue(
        gotPiece > 0,
        "World physics failed to evaluate checkIntake properly; game piece should have been intaked.");
    assertTrue(
        targetPiece.isIntaked(),
        "Piece should be marked intaked internally after successful evaluation.");
  }

  @Test
  public void testStaleWarningDetectionAndReset() {
    MARSPhysicsWorld world = MARSPhysicsWorld.getInstance();

    int initialCount = world.getBodyCount();

    // Add dummy bodies to cross the STALE_BODY_THRESHOLD (20)
    for (int i = 0; i < 25; i++) {
      org.dyn4j.dynamics.Body dummy = new org.dyn4j.dynamics.Body();
      world.getDyn4jWorld().addBody(dummy);
    }

    assertTrue(
        world.getBodyCount() > initialCount, "Should have more than 20 bodies for stale testing");

    // Hit the getInstance() threshold (500 accesses without a reset)
    for (int i = 0; i < 505; i++) {
      MARSPhysicsWorld.getInstance();
    }

    // Ensure the resetInstance completely wipes the world and the access count
    com.marslib.testing.MARSTestHarness.reset();
    MARSPhysicsWorld newWorld = MARSPhysicsWorld.getInstance();

    assertEquals(
        initialCount,
        newWorld.getBodyCount(),
        "New world should not have inherited the dummy bodies");
  }
}
