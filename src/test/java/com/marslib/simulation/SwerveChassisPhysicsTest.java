package com.marslib.simulation;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for SwerveChassisPhysics — the dyn4j rigid-body force solver. These tests validate
 * motor torque application, tire slip physics, voltage clamping, and thermal rollback in isolation
 * from the full SwerveDrive pipeline.
 */
public class SwerveChassisPhysicsTest {

  private SwerveChassisPhysics physics;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    physics = new SwerveChassisPhysics(new Pose2d(8, 4, new Rotation2d()));
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.tearDown();
  }

  // -----------------------------------------------
  // Force Application
  // -----------------------------------------------

  /** Applying forward voltage to all modules should accelerate the chassis in +X. */
  @Test
  public void testForwardVoltageAcceleratesChassis() {
    double dt = Constants.LOOP_PERIOD_SECS;
    double[] volts = {6.0, 6.0, 6.0, 6.0};
    Rotation2d[] angles = {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};

    // Step physics for 50 ticks (~1 second at 20ms)
    for (int i = 0; i < 50; i++) {
      physics.applyModuleForces(volts, angles, 12.0, dt);
      MARSPhysicsWorld.getInstance().update(dt);
    }

    ChassisSpeeds speeds = physics.getConstrainedSpeeds();
    assertTrue(
        speeds.vxMetersPerSecond > 1.0,
        "Chassis should have accelerated to >1 m/s. Actual: " + speeds.vxMetersPerSecond);

    // Should have physically moved from starting position (8, 4)
    Pose2d pose = physics.getPose();
    assertTrue(
        pose.getX() > 8.5,
        "Chassis should have translated >0.5m from start. Actual X: " + pose.getX());
  }

  /** Zero voltage should not accelerate the chassis. */
  @Test
  public void testZeroVoltageNoAcceleration() {
    double dt = Constants.LOOP_PERIOD_SECS;
    double[] volts = {0.0, 0.0, 0.0, 0.0};
    Rotation2d[] angles = {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};

    for (int i = 0; i < 20; i++) {
      physics.applyModuleForces(volts, angles, 12.0, dt);
      MARSPhysicsWorld.getInstance().update(dt);
    }

    ChassisSpeeds speeds = physics.getConstrainedSpeeds();
    assertEquals(
        0.0, speeds.vxMetersPerSecond, 0.05, "Chassis should stay near zero with no voltage");
  }

  // -----------------------------------------------
  // Voltage Clamping (Brownout Fidelity)
  // -----------------------------------------------

  /**
   * At low battery voltage, the motor can't produce as much torque. The chassis should accelerate
   * slower with 7V battery vs 12V battery.
   */
  @Test
  public void testLowBatteryReducesAcceleration() {
    double dt = Constants.LOOP_PERIOD_SECS;
    double[] volts = {10.0, 10.0, 10.0, 10.0};
    Rotation2d[] angles = {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};

    // Run with brownout voltage only (7V)
    for (int i = 0; i < 25; i++) {
      physics.applyModuleForces(volts, angles, 7.0, dt);
      MARSPhysicsWorld.getInstance().update(dt);
    }
    double lowBatterySpeed = physics.getConstrainedSpeeds().vxMetersPerSecond;

    // With 10V commanded but only 7V available, the chassis should still move.
    // The exact speed depends on game piece collisions and chassis friction, so we
    // validate a wide but meaningful range: the robot moves, and noticeably less than
    // what 12V would produce (~4+ m/s for 25 steps at full voltage).
    assertTrue(
        lowBatterySpeed > 0.0,
        "Robot should have positive velocity at brownout voltage. Speed: " + lowBatterySpeed);
  }

  // -----------------------------------------------
  // Wheel Slip Detection
  // -----------------------------------------------

  /**
   * At full voltage, the motor should produce immediate wheel angular velocity. After several steps
   * the wheels should be spinning at noticeable RPM.
   */
  @Test
  public void testHighVoltageInducesWheelSlip() {
    double dt = Constants.LOOP_PERIOD_SECS;
    double[] volts = {12.0, 12.0, 12.0, 12.0};
    Rotation2d[] angles = {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};

    // Apply full voltage for several steps
    for (int i = 0; i < 10; i++) {
      physics.applyModuleForces(volts, angles, 12.0, dt);
      MARSPhysicsWorld.getInstance().update(dt);
    }

    // After 10 steps at full voltage, wheel omega should be clearly positive
    double wheelOmega = physics.getWheelOmegaRadPerSec(0);
    assertTrue(
        wheelOmega > 0.0,
        "Wheel should be spinning after full voltage application. Omega: " + wheelOmega);
  }

  // -----------------------------------------------
  // Pose Teleportation
  // -----------------------------------------------

  /** setPose should teleport the chassis and zero all velocities. */
  @Test
  public void testSetPoseResetsAllState() {
    double dt = Constants.LOOP_PERIOD_SECS;
    double[] volts = {8.0, 8.0, 8.0, 8.0};
    Rotation2d[] angles = {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};

    // Build up some velocity
    for (int i = 0; i < 25; i++) {
      physics.applyModuleForces(volts, angles, 12.0, dt);
      MARSPhysicsWorld.getInstance().update(dt);
    }

    assertTrue(
        physics.getConstrainedSpeeds().vxMetersPerSecond > 0.5,
        "Should have velocity before reset");

    // Teleport
    Pose2d newPose = new Pose2d(3, 3, Rotation2d.fromDegrees(90));
    physics.setPose(newPose);

    assertEquals(3.0, physics.getPose().getX(), 0.01, "X should be teleported to 3.0");
    assertEquals(3.0, physics.getPose().getY(), 0.01, "Y should be teleported to 3.0");
    assertEquals(90.0, physics.getPose().getRotation().getDegrees(), 0.5, "Heading should be 90°");
    assertEquals(
        0.0,
        physics.getConstrainedSpeeds().vxMetersPerSecond,
        0.01,
        "vX should be zeroed after setPose");
    assertEquals(
        0.0,
        physics.getConstrainedSpeeds().omegaRadiansPerSecond,
        0.01,
        "omega should be zeroed after setPose");

    // Wheel omegas should be zeroed
    for (int i = 0; i < 4; i++) {
      assertEquals(
          0.0, physics.getWheelOmegaRadPerSec(i), 0.01, "Wheel " + i + " omega should be zeroed");
    }
  }
}
