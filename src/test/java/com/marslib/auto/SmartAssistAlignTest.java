package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.marslib.swerve.SwerveDrive;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class SmartAssistAlignTest {

  private SwerveDrive mockSwerveDrive;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    mockSwerveDrive = mock(SwerveDrive.class);
  }

  @Test
  public void testSmartAssistAllowsXMovementButAutomatesYAndTheta() {
    Pose2d currentPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d targetNode = new Pose2d(3, 3, Rotation2d.fromDegrees(90));

    when(mockSwerveDrive.getPose()).thenReturn(currentPose);

    // Driver is pushing forward at 2.0 m/s
    SmartAssistAlign command = new SmartAssistAlign(mockSwerveDrive, () -> 2.0, targetNode);

    command.initialize();
    command.execute();

    ArgumentCaptor<ChassisSpeeds> speedsCaptor = ArgumentCaptor.forClass(ChassisSpeeds.class);
    verify(mockSwerveDrive).runVelocity(speedsCaptor.capture());

    ChassisSpeeds commanded = speedsCaptor.getValue();

    // Because current heading is 0, Field-Relative translation maps directly:
    // Field X = Robot X, Field Y = Robot Y.
    // The driver commanded 2.0 in X.
    assertEquals(
        2.0,
        commanded.vxMetersPerSecond,
        0.001,
        "Driver forward speed should be perfectly passed through.");

    // The target is at Y=3, robot is at Y=0. It should command positive translation in Y.
    assertTrue(
        commanded.vyMetersPerSecond > 0.0, "Should automatically strafe leftward toward target Y.");

    // The target theta is 90 degrees, robot is at 0. It should command positive rotation.
    assertTrue(
        commanded.omegaRadiansPerSecond > 0.0, "Should automatically rotate toward target theta.");
  }
}
