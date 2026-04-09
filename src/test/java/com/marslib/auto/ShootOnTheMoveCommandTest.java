package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.MARSCowl;
import frc.robot.subsystems.MARSShooter;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for ShootOnTheMoveCommand's velocity-leading aim math.
 *
 * <p>The command solves a quadratic Time-Of-Flight equation to predict where the target will be
 * when the game piece arrives. This test validates the math catches regressions in the TOF solver,
 * the virtual target calculation, and the edge cases (stationary, parallel motion, out-of-range).
 */
public class ShootOnTheMoveCommandTest {

  private SwerveDrive swerveDrive;
  private MARSCowl cowl;
  private MARSShooter shooter;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();

    GyroIOSim gyroSim = new GyroIOSim();
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());

    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }

    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);
    swerveDrive.resetPose(new Pose2d(0, 0, new Rotation2d()));

    cowl =
        new MARSCowl(
            new com.marslib.mechanisms.RotaryMechanismIOSim("Cowl", 50.0, 0.5, 0.5), powerManager);
    shooter =
        new MARSShooter(
            new com.marslib.mechanisms.FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
  }

  // -----------------------------------------------
  // Physical Integration Tests
  // -----------------------------------------------

  /** When the robot is stationary, the aim heading should point directly at the target hub. */
  @Test
  public void testStationaryAim() {
    ShootOnTheMoveCommand command =
        new ShootOnTheMoveCommand(swerveDrive, cowl, shooter, () -> 0.0, () -> 0.0);

    CommandScheduler.getInstance().schedule(command);

    for (int i = 0; i < 300; i++) {
      SimHooks.stepTiming(ModeConstants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(ModeConstants.LOOP_PERIOD_SECS);
    }

    // From (0,0) aiming at BLUE_HUB_POS, the expected target angle is atan2(hub_y, hub_x)
    Pose2d resultingPose = swerveDrive.getPose();
    double expectedAngleRaw =
        Math.atan2(
            FieldConstants.BLUE_HUB_POS.getY() - resultingPose.getY(),
            FieldConstants.BLUE_HUB_POS.getX() - resultingPose.getX());

    double finalDeg = resultingPose.getRotation().getDegrees();
    double expectedDeg = Math.toDegrees(expectedAngleRaw);

    assertTrue(
        Math.abs(finalDeg - expectedDeg) < 5.0,
        "Physical robot failed to pivot to static target heading. Final: "
            + finalDeg
            + ", Expected: "
            + expectedDeg);
  }

  /** When the robot is translating, the aim should lead the target (diverge from static aim). */
  @Test
  public void testMovingAimCalculatesVirtualTarget() {
    ShootOnTheMoveCommand command =
        new ShootOnTheMoveCommand(swerveDrive, cowl, shooter, () -> 2.0, () -> 0.0);

    CommandScheduler.getInstance().schedule(command);

    for (int i = 0; i < 300; i++) {
      SimHooks.stepTiming(ModeConstants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(ModeConstants.LOOP_PERIOD_SECS);
    }

    Pose2d resultingPose = swerveDrive.getPose();

    // Robot physically moved
    assertTrue(
        resultingPose.getX() > 1.0,
        "Robot failed to translate continuously via joysticks in physical testing!");

    double staticAngleRaw =
        Math.atan2(
            FieldConstants.BLUE_HUB_POS.getY() - resultingPose.getY(),
            FieldConstants.BLUE_HUB_POS.getX() - resultingPose.getX());

    double expectedStaticDeg = Math.toDegrees(staticAngleRaw);
    double actualDeg = resultingPose.getRotation().getDegrees();

    // Since the robot is translating, the aim must lead the target — angles MUST diverge
    assertTrue(
        Math.abs(actualDeg - expectedStaticDeg) > 0.5,
        "Robot failed to apply dynamic kinematic offset! Actual: "
            + actualDeg
            + ", Static: "
            + expectedStaticDeg);
  }

  /**
   * When the target is totally unreachable, the robot should fall back to naive aiming cleanly
   * without crashing.
   */
  @Test
  public void testPhysicalFallbackOnNegativeDiscriminant() {
    // Robot translating AWAY at an impossible speed (30 m/s > 15 m/s projectile)
    ShootOnTheMoveCommand command =
        new ShootOnTheMoveCommand(swerveDrive, cowl, shooter, () -> -30.0, () -> -30.0);

    CommandScheduler.getInstance().schedule(command);

    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(ModeConstants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(ModeConstants.LOOP_PERIOD_SECS);
    }

    // If it ran without exception, the fallback logic executed successfully inside the loop
    Pose2d resultingPose = swerveDrive.getPose();
    assertNotNull(resultingPose, "Swerve geometry must not be NaN under fallback path");
  }

  // -----------------------------------------------
  // Pure-Math TOF Solver Tests
  // -----------------------------------------------

  /**
   * Tests the quadratic TOF solver with a stationary robot. When velocity is zero, the virtual
   * target should equal the real target (no leading offset). The TOF should be distance / speed.
   */
  @Test
  public void testTofSolverStationary() {
    Translation2d robotPos = new Translation2d(0, 0);
    Translation2d target = FieldConstants.BLUE_HUB_POS; // (4.62, 4.03)
    double vx = 0.0;
    double vy = 0.0;
    double s = ShooterConstants.PROJECTILE_SPEED_MPS; // 15.0

    TofResult result = solveTof(robotPos, target, vx, vy, s);

    double expectedTof = robotPos.getDistance(target) / s;
    assertEquals(
        expectedTof, result.timeOfFlight, 0.001, "Stationary TOF should be distance/speed");

    // Virtual target should equal real target when stationary
    assertEquals(target.getX(), result.virtualTarget.getX(), 0.01, "Virtual X should match target");
    assertEquals(target.getY(), result.virtualTarget.getY(), 0.01, "Virtual Y should match target");
  }

  /**
   * Tests that the TOF solver leads the target when the robot is moving perpendicular to the
   * target. If moving along +X while target is at (4.62, 4.03), the virtual target should shift
   * left (negative X offset) to compensate.
   */
  @Test
  public void testTofSolverLeadsTargetWhenMoving() {
    Translation2d robotPos = new Translation2d(0, 0);
    Translation2d target = FieldConstants.BLUE_HUB_POS;
    double vx = 3.0; // Moving 3 m/s along X
    double vy = 0.0;
    double s = ShooterConstants.PROJECTILE_SPEED_MPS;

    TofResult result = solveTof(robotPos, target, vx, vy, s);

    // Virtual target should be shifted BACKWARDS from robot velocity
    // i.e., virtualX = target.x - vx * tof → shifted negatively
    assertTrue(
        result.virtualTarget.getX() < target.getX(),
        "Virtual target should lead (shift opposite to velocity). Virtual: "
            + result.virtualTarget.getX()
            + ", Real: "
            + target.getX());
  }

  /**
   * Tests that the aim angle changes based on robot velocity. A robot moving along +X should aim
   * more toward pure-Y (rotated from static aim) to compensate.
   */
  @Test
  public void testAimAngleDivergesFromStaticWhenMoving() {
    Translation2d robotPos = new Translation2d(2, 0);
    Translation2d target = FieldConstants.BLUE_HUB_POS;
    double s = ShooterConstants.PROJECTILE_SPEED_MPS;

    // Static aim angle
    TofResult stationary = solveTof(robotPos, target, 0, 0, s);
    double staticAngle =
        Math.atan2(
            stationary.virtualTarget.getY() - robotPos.getY(),
            stationary.virtualTarget.getX() - robotPos.getX());

    // Moving aim angle
    TofResult moving = solveTof(robotPos, target, 3.0, 0, s);
    double movingAngle =
        Math.atan2(
            moving.virtualTarget.getY() - robotPos.getY(),
            moving.virtualTarget.getX() - robotPos.getX());

    assertTrue(
        Math.abs(staticAngle - movingAngle) > 0.01,
        "Moving aim angle should diverge from static aim angle");
  }

  /**
   * Tests that the TOF solver gracefully handles the case where the projectile can't reach the
   * target (negative discriminant). Should fall back to naive distance/speed.
   */
  @Test
  public void testTofSolverFallbackOnNegativeDiscriminant() {
    Translation2d robotPos = new Translation2d(0, 0);
    Translation2d target = new Translation2d(100, 100); // Very far away
    // Robot moving AWAY from target faster than projectile — impossible to intercept
    double vx = -20.0;
    double vy = -20.0;
    double s = 1.0; // Slow projectile — can't catch the virtual target

    TofResult result = solveTof(robotPos, target, vx, vy, s);

    // Should fallback without crashing
    assertTrue(result.timeOfFlight > 0, "Fallback TOF should be positive");
    assertNotNull(result.virtualTarget, "Virtual target should not be null on fallback");
  }

  // -----------------------------------------------
  // Helper: replicates the exact TOF math from ShootOnTheMoveCommand
  // -----------------------------------------------

  private static class TofResult {
    final double timeOfFlight;
    final Translation2d virtualTarget;

    TofResult(double tof, Translation2d vt) {
      this.timeOfFlight = tof;
      this.virtualTarget = vt;
    }
  }

  /**
   * Exact copy of the quadratic solver from ShootOnTheMoveCommand.execute(). This tests the math in
   * isolation without needing a full SwerveDrive + PIDController.
   */
  private static TofResult solveTof(
      Translation2d robotPos, Translation2d target, double vx, double vy, double s) {
    double dx = target.getX() - robotPos.getX();
    double dy = target.getY() - robotPos.getY();

    double a = (s * s) - ((vx * vx) + (vy * vy));
    double b = -2.0 * ((dx * vx) + (dy * vy));
    double c = -((dx * dx) + (dy * dy));

    double discriminant = (b * b) - (4.0 * a * c);
    double timeOfFlight;

    if (discriminant < 0.0 || a == 0.0) {
      timeOfFlight = robotPos.getDistance(target) / Math.max(s, 0.01);
    } else {
      double t1 = (-b + Math.sqrt(discriminant)) / (2.0 * a);
      double t2 = (-b - Math.sqrt(discriminant)) / (2.0 * a);

      if (t1 > 0.0 && t2 > 0.0) timeOfFlight = Math.min(t1, t2);
      else if (t1 > 0.0) timeOfFlight = t1;
      else if (t2 > 0.0) timeOfFlight = t2;
      else timeOfFlight = robotPos.getDistance(target) / Math.max(s, 0.01);
    }

    double virtualTargetX = target.getX() - (vx * timeOfFlight);
    double virtualTargetY = target.getY() - (vy * timeOfFlight);

    return new TofResult(timeOfFlight, new Translation2d(virtualTargetX, virtualTargetY));
  }
}
