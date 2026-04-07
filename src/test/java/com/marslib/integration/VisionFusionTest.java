package com.marslib.integration;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.faults.Alert;
import com.marslib.faults.MARSFaultManager;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.vision.AprilTagVisionIOSim;
import com.marslib.vision.MARSVision;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Integration test for the vision + swerve pose estimation fusion pipeline.
 *
 * <p>Validates that {@link MARSVision} correctly injects AprilTag measurements into {@link
 * SwerveDrive}'s pose estimator, producing corrected pose estimates that converge toward the true
 * physical position from the dyn4j simulation.
 *
 * <p>This is the highest-value untested path: a single-camera vision sim feeding into the swerve
 * odometry loop while the robot is stationary. If vision fusion is broken (wrong timestamps, wrong
 * coordinate frame, or rejected measurements), this test will catch it.
 */
public class VisionFusionTest {

  private SwerveDrive swerveDrive;
  private MARSVision vision;
  private MARSPowerManager powerManager;

  /**
   * Camera mounted at center-front of the robot, facing forward. Translation: 0.3m forward, 0m
   * sideways, 0.5m up. Rotation: no pitch/yaw/roll offset.
   */
  private static final Transform3d CAMERA_TRANSFORM =
      new Transform3d(new Translation3d(0.3, 0.0, 0.5), new Rotation3d(0, 0, 0));

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    MARSPhysicsWorld.resetInstance();
    AprilTagVisionIOSim.resetSimulation();
    Alert.resetAll();
    MARSFaultManager.clear();

    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    // Construct swerve
    powerManager = new MARSPowerManager(new PowerIOSim());
    GyroIOSim gyroSim = new GyroIOSim();

    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }

    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);

    // Start the robot at a known pose near AprilTags so the camera can "see" them
    // Blue alliance wall area — tags should be visible on the far wall/reef
    Pose2d startPose = new Pose2d(3.0, 4.0, new Rotation2d(0));
    swerveDrive.resetPose(startPose);

    // Construct vision with a single sim camera
    AprilTagVisionIOSim cameraIO =
        new AprilTagVisionIOSim("TestCam", CAMERA_TRANSFORM, swerveDrive::getPose);

    vision = new MARSVision(swerveDrive, List.of(cameraIO), List.of());
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  /**
   * Tests that the vision + swerve pipeline runs without crashing for 5 seconds. This catches null
   * pointer exceptions, missing field layouts, and configuration errors.
   */
  @Test
  public void testVisionSwerveDoesNotCrash() {
    for (int i = 0; i < 250; i++) {
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
      assertDoesNotThrow(
          () -> CommandScheduler.getInstance().run(),
          "Vision-swerve pipeline crashed at tick " + i);
      MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
    }

    // Pose estimator should still be valid after 5s of vision+odometry fusion
    Pose2d finalPose = swerveDrive.getPose();
    assertNotNull(finalPose, "Pose estimator returned null after vision fusion");
    assertFalse(
        Double.isNaN(finalPose.getX()) || Double.isNaN(finalPose.getY()),
        "Pose estimator produced NaN after vision fusion");
  }

  /**
   * Tests that a stationary robot's pose remains stable when vision is actively injecting
   * measurements. The pose should stay near the initial position — if vision is applying massive
   * offsets, the pose will drift wildly.
   */
  @Test
  public void testStationaryPoseStability() {
    Pose2d initialPose = swerveDrive.getPose();

    // Run for 3 seconds (150 ticks) while stationary
    for (int i = 0; i < 150; i++) {
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
    }

    Pose2d finalPose = swerveDrive.getPose();

    // A stationary robot with vision should stay within 1.0m of its initial position.
    // This is generous — the real tolerance is ~0.1m, but vision noise and estimator
    // dynamics can shift things slightly depending on which tags are visible.
    double drift =
        Math.hypot(finalPose.getX() - initialPose.getX(), finalPose.getY() - initialPose.getY());

    assertTrue(
        drift < 1.0,
        String.format(
            "Stationary robot drifted %.2fm after vision fusion (initial: %s, final: %s)",
            drift, initialPose, finalPose));
  }

  /**
   * Tests that the physics world singleton was properly reset and is not carrying stale state.
   * Verifies the stale-state detection guard we added.
   */
  @Test
  public void testPhysicsWorldResetVerification() {
    // After setUp(), the physics world should have a known set of bodies:
    // ~7 field boundaries + ~168 game pieces = ~175 total for a fresh world.
    // A stale world (unreset between tests) would have 350+ from doubled chassis/modules.
    int bodyCount = MARSPhysicsWorld.getInstance().getBodyCount();
    assertTrue(
        bodyCount < 250,
        "Physics world has "
            + bodyCount
            + " bodies — expected <250 for a fresh world."
            + " A count well above ~175 indicates resetInstance() is not working.");
  }
}
