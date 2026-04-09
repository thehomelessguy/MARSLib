package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.constants.DriveConstants;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for the teleop drive pipeline math: deadband → cube → scale → alliance-flip → slew.
 *
 * <p>This test calls {@link TeleopDriveMath#computeFieldRelativeSpeeds} directly — the exact same
 * method used by {@code RobotContainer}. A sign error in alliance flipping, deadband, or cubic
 * scaling would make the robot drive backward or steer wildly at competition.
 */
public class TeleopDrivePipelineTest {

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
  }

  // -----------------------------------------------
  // Deadband Tests
  // -----------------------------------------------

  /** Joystick values within deadband should produce zero output. */
  @Test
  public void testDeadbandFiltersSmallInputs() {
    ChassisSpeeds speeds = TeleopDriveMath.computeFieldRelativeSpeeds(0.05, -0.05, 0.03, false);
    assertEquals(0.0, speeds.vxMetersPerSecond, 0.001, "VX should be 0 within deadband");
    assertEquals(0.0, speeds.vyMetersPerSecond, 0.001, "VY should be 0 within deadband");
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 0.001, "Omega should be 0 within deadband");
  }

  /** Zero joystick input should produce zero output. */
  @Test
  public void testZeroInputProducesZeroOutput() {
    ChassisSpeeds speeds = TeleopDriveMath.computeFieldRelativeSpeeds(0.0, 0.0, 0.0, false);
    assertEquals(0.0, speeds.vxMetersPerSecond, 0.001);
    assertEquals(0.0, speeds.vyMetersPerSecond, 0.001);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 0.001);
  }

  // -----------------------------------------------
  // Full Deflection Tests
  // -----------------------------------------------

  /**
   * Full forward stick (leftY = -1.0 in WPILib) should produce positive VX on Blue alliance. The
   * WPILib convention is: stick forward = negative axis value, but field-relative forward =
   * positive X. The pipeline applies a negate to convert.
   */
  @Test
  public void testFullForwardBlueAlliance() {
    ChassisSpeeds speeds = TeleopDriveMath.computeFieldRelativeSpeeds(-1.0, 0.0, 0.0, false);
    assertTrue(
        speeds.vxMetersPerSecond > 0,
        "Full forward on Blue should produce positive VX, got " + speeds.vxMetersPerSecond);
    assertEquals(
        SwerveConstants.MAX_LINEAR_SPEED_MPS,
        speeds.vxMetersPerSecond,
        0.001,
        "Full forward should reach max speed");
  }

  /** Full forward on Red alliance should produce NEGATIVE VX (field-flipped). */
  @Test
  public void testFullForwardRedAllianceFlipsSign() {
    ChassisSpeeds speeds = TeleopDriveMath.computeFieldRelativeSpeeds(-1.0, 0.0, 0.0, true);
    assertTrue(
        speeds.vxMetersPerSecond < 0,
        "Full forward on Red should produce negative VX (flipped), got "
            + speeds.vxMetersPerSecond);
    assertEquals(
        -SwerveConstants.MAX_LINEAR_SPEED_MPS,
        speeds.vxMetersPerSecond,
        0.001,
        "Full forward Red should be -MAX_SPEED");
  }

  /**
   * Alliance flipping should apply to both X and Y axes. Full left stick on Red should be opposite
   * sign to Blue.
   */
  @Test
  public void testAllianceFlipAffectsBothAxes() {
    ChassisSpeeds blue = TeleopDriveMath.computeFieldRelativeSpeeds(-1.0, -1.0, 0.0, false);
    ChassisSpeeds red = TeleopDriveMath.computeFieldRelativeSpeeds(-1.0, -1.0, 0.0, true);

    assertEquals(
        -blue.vxMetersPerSecond,
        red.vxMetersPerSecond,
        0.001,
        "Red VX should be negation of Blue VX");
    assertEquals(
        -blue.vyMetersPerSecond,
        red.vyMetersPerSecond,
        0.001,
        "Red VY should be negation of Blue VY");
  }

  /** Alliance flip should NOT affect rotation — rotation is always robot-relative. */
  @Test
  public void testAllianceFlipDoesNotAffectRotation() {
    ChassisSpeeds blue = TeleopDriveMath.computeFieldRelativeSpeeds(0.0, 0.0, 1.0, false);
    ChassisSpeeds red = TeleopDriveMath.computeFieldRelativeSpeeds(0.0, 0.0, 1.0, true);

    assertEquals(
        blue.omegaRadiansPerSecond,
        red.omegaRadiansPerSecond,
        0.001,
        "Rotation should be identical regardless of alliance");
  }

  // -----------------------------------------------
  // Cubic Scaling Tests
  // -----------------------------------------------

  /**
   * Cubic scaling should provide fine control at small deflections. The deadband rescales the axis
   * first (e.g., -0.5 with 0.1 deadband becomes -0.444), THEN the rescaled value is cubed.
   */
  @Test
  public void testCubicScalingAtHalfDeflection() {
    ChassisSpeeds speeds = TeleopDriveMath.computeFieldRelativeSpeeds(-0.5, 0.0, 0.0, false);
    // After deadband rescale: (-0.5 - (-0.1)) / (1.0 - 0.1) = -0.4/0.9 ≈ -0.4444
    double postDeadband = MathUtil.applyDeadband(-0.5, TeleopDriveMath.DEADBAND);
    double expected = Math.pow(Math.abs(postDeadband), 3.0) * SwerveConstants.MAX_LINEAR_SPEED_MPS;
    assertEquals(expected, speeds.vxMetersPerSecond, 0.01, "Cubic scaling at 50% deflection");
  }

  // -----------------------------------------------
  // Slew Rate Limiter Tests
  // -----------------------------------------------

  /**
   * Slew rate limiters should prevent instantaneous acceleration. Going from 0 to max speed in one
   * tick should be capped.
   */
  @Test
  public void testSlewRateLimiterCapsAcceleration() {
    SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_LINEAR_ACCEL_LIMIT);

    // First tick: from 0 to max speed
    double requestedSpeed = SwerveConstants.MAX_LINEAR_SPEED_MPS;
    double limitedSpeed = xLimiter.calculate(requestedSpeed);

    // The limiter should cap the output to dt * rateLimit (20ms * 15 m/s^2 = 0.3 m/s)
    assertTrue(
        limitedSpeed < requestedSpeed,
        String.format(
            "Slew limiter should cap acceleration. Requested: %.2f, Got: %.2f",
            requestedSpeed, limitedSpeed));
    assertTrue(limitedSpeed > 0, "Slew limiter should still allow some movement");
  }
}
