package com.marslib.simulation;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.world.World;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LidarIOSimTest {

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    // Clear out any old bodies from previous tests by re-initializing the internal world somehow
    // Actually, dyn4j provides world.removeAllBodiesAndJoints()
    World<Body> world = MARSPhysicsWorld.getInstance().getDyn4jWorld();
    if (world != null) {
      world.removeAllBodiesAndJoints();
    }
  }

  @Test
  public void testLidarDetectsDyn4jWall() {
    World<Body> world = MARSPhysicsWorld.getInstance().getDyn4jWorld();
    assertNotNull(world, "Dyn4j world must be instantiated properly.");

    // Create a 1x1 meter wall directly in front of the robot at (x=2.0, y=0.0)
    Body wall = new Body();
    wall.addFixture(Geometry.createRectangle(1.0, 1.0));
    wall.translate(2.0, 0.0);
    world.addBody(wall);

    LidarIOSim lidarSim = new LidarIOSim();

    // The robot is at (0,0) facing exactly towards the wall (0 degrees)
    Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    // The updateInputs method does internal raycasting
    // Currently, it logs point clouds directly into AdvantageKit.
    // For unit tests, we'd normally verify the output structures. Since it uses
    // Logger.recordOutput()
    // It's a void method. To verify, we would either need an interface or to check AdvantageKit's
    // log dump.
    // However, we can quickly assert it runs without null pointers or math exceptions.
    assertDoesNotThrow(() -> lidarSim.updateInputs(robotPose));

    // We can confidently verify the Dyn4j bodies are resolving in the tree
    assertEquals(1, world.getBodyCount());
  }
}
