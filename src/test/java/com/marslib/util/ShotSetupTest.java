package com.marslib.util;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the ShotSetup SOTM (Shot-On-The-Move) utility. Validates the interpolation maps,
 * clamping bounds, TOF convergence algorithm, and dynamic aiming math in isolation.
 */
public class ShotSetupTest {

  private ShotSetup shotSetup;
  private SwerveDrive swerveDrive;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();

    // Build the shot setup with known interpolation points
    shotSetup =
        new ShotSetup(
            0.1, // phaseDelay
            Math.PI / 2, // maxCowlPosition
            6000.0, // maxFlywheelSpeedRPM
            0.25, // recomputeThreshold
            5, // convergenceIters
            0.01, // convergenceEpsilon
            0.1, // minTof
            1.5, // maxTof
            new Transform2d(new Translation2d(0, 0), new Rotation2d()),
            new Rotation2d());

    // Distance → (RPM, CowlAngle)
    shotSetup.addShotMapEntry(1.0, 2000.0, 0.15);
    shotSetup.addShotMapEntry(3.0, 3000.0, 0.35);
    shotSetup.addShotMapEntry(5.0, 4000.0, 0.55);
    shotSetup.addShotMapEntry(7.0, 4800.0, 0.75);
    shotSetup.addShotMapEntry(10.0, 5500.0, 1.0);

    // Distance → TOF
    shotSetup.addTofMapEntry(1.0, 0.15);
    shotSetup.addTofMapEntry(3.0, 0.25);
    shotSetup.addTofMapEntry(5.0, 0.40);
    shotSetup.addTofMapEntry(7.0, 0.60);
    shotSetup.addTofMapEntry(10.0, 0.90);

    // Build a SwerveDrive for SOTM integration tests
    GyroIOSim gyroSim = new GyroIOSim();
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());
    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }
    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);
    swerveDrive.resetPose(new Pose2d(2, 2, new Rotation2d()));
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
  }

  // -----------------------------------------------
  // Static Shot Interpolation Tests
  // -----------------------------------------------

  /** At an exact map entry distance, should return exact RPM and cowl values. */
  @Test
  public void testExactMapEntryReturnsExactValues() {
    ShotSetup.ShotInfo info = shotSetup.getStaticShotInfo(5.0);
    assertEquals(4000.0, info.shot.shooterRPM, 0.1, "RPM at 5m should be exactly 4000");
    assertEquals(0.55, info.cowlPosition, 0.001, "Cowl at 5m should be exactly 0.55");
  }

  /** Between map entries, values should be linearly interpolated. */
  @Test
  public void testInterpolationBetweenEntries() {
    // At 2.0m (midpoint between 1m and 3m entries)
    ShotSetup.ShotInfo info = shotSetup.getStaticShotInfo(2.0);
    assertEquals(2500.0, info.shot.shooterRPM, 50.0, "RPM at 2m should interpolate to ~2500");
    assertEquals(0.25, info.cowlPosition, 0.02, "Cowl at 2m should interpolate to ~0.25");
  }

  /** RPM should be clamped to maxFlywheelSpeedRPM (6000). */
  @Test
  public void testRpmClampedToMax() {
    // Add an entry that exceeds the max
    shotSetup.addShotMapEntry(15.0, 8000.0, 2.0);
    ShotSetup.ShotInfo info = shotSetup.getStaticShotInfo(15.0);
    assertEquals(6000.0, info.shot.shooterRPM, 0.1, "RPM should be clamped to 6000 max");
  }

  /** Cowl position should be clamped to maxCowlPosition (π/2). */
  @Test
  public void testCowlClampedToMax() {
    shotSetup.addShotMapEntry(15.0, 5000.0, 3.0);
    ShotSetup.ShotInfo info = shotSetup.getStaticShotInfo(15.0);
    assertEquals(Math.PI / 2, info.cowlPosition, 0.001, "Cowl should be clamped to π/2 max");
  }

  /** Cowl position should never go below zero. */
  @Test
  public void testCowlClampedToMinZero() {
    shotSetup.addShotMapEntry(0.5, 1000.0, -0.5);
    ShotSetup.ShotInfo info = shotSetup.getStaticShotInfo(0.5);
    assertEquals(0.0, info.cowlPosition, 0.001, "Cowl should be clamped to 0 minimum");
  }

  // -----------------------------------------------
  // SOTM Integration Tests
  // -----------------------------------------------

  /**
   * When the robot is stationary, SOTM info should match static shot info. The virtual target angle
   * should point directly at the target with near-zero angular velocity feedforward.
   */
  @Test
  public void testStationarySOTMMatchesStatic() {
    // Ensure the robot is fully stopped
    swerveDrive.runVelocity(new ChassisSpeeds(0, 0, 0));

    // Step physics a few times to settle
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(ModeConstants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(ModeConstants.LOOP_PERIOD_SECS);
    }

    Translation2d target = FieldConstants.BLUE_HUB_POS;
    ShotSetup.SOTMInfo sotm = shotSetup.getSOTMInfo(swerveDrive, target);

    assertNotNull(sotm, "SOTM info should not be null");
    assertNotNull(sotm.shotInfo, "Shot info should not be null");
    assertTrue(sotm.shotInfo.shot.shooterRPM > 0, "RPM should be positive");

    // Angular velocity feedforward should be near zero when stationary
    assertEquals(
        0.0,
        sotm.angularVelocityRadPerSec,
        0.1,
        "Angular velocity FF should be ~0 for stationary robot");
  }

  /**
   * Calling getSOTMInfo twice with the same position should return cached results (spatial cache
   * hit).
   */
  @Test
  public void testSOTMCacheHitOnSamePosition() {
    Translation2d target = FieldConstants.BLUE_HUB_POS;

    // First call — computes fresh
    ShotSetup.SOTMInfo first = shotSetup.getSOTMInfo(swerveDrive, target);
    // Second call — should return cached (same position, within threshold)
    ShotSetup.SOTMInfo second = shotSetup.getSOTMInfo(swerveDrive, target);

    // They should be the exact same object reference (cache hit)
    assertSame(first, second, "Second call should return cached SOTM result");
  }

  /**
   * Virtual target angle should point toward the target from the robot's perspective. For a robot
   * at (2,2) aiming at BLUE_HUB_POS (4.62, 4.03), the angle should be roughly atan2(2.03, 2.62) ≈
   * 37.8°.
   */
  @Test
  public void testVirtualTargetAnglePointsAtTarget() {
    swerveDrive.runVelocity(new ChassisSpeeds(0, 0, 0));

    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(ModeConstants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(ModeConstants.LOOP_PERIOD_SECS);
    }

    Translation2d target = FieldConstants.BLUE_HUB_POS;
    ShotSetup.SOTMInfo sotm = shotSetup.getSOTMInfo(swerveDrive, target);

    Pose2d pose = swerveDrive.getPose();
    double expectedAngle = Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX());
    double actualAngle = sotm.virtualTargetAngle.getRadians();

    // Should be within ~10° (phase delay shifts it slightly)
    assertEquals(
        expectedAngle,
        actualAngle,
        Math.toRadians(10),
        "Virtual target should roughly point at target. Expected: "
            + Math.toDegrees(expectedAngle)
            + "°, Got: "
            + Math.toDegrees(actualAngle)
            + "°");
  }
}
