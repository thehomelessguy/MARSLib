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
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SmartAssistAlignTest {

  private SwerveDrive swerveDrive;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());

    swerveDrive =
        new SwerveDrive(
            new SwerveModule[] {
              new SwerveModule(0, new SwerveModuleIOSim(0)),
              new SwerveModule(1, new SwerveModuleIOSim(1)),
              new SwerveModule(2, new SwerveModuleIOSim(2)),
              new SwerveModule(3, new SwerveModuleIOSim(3))
            },
            new GyroIOSim(),
            powerManager);
  }

  @Test
  public void testSmartAssistAllowsXMovementButAutomatesYAndTheta() {
    // Current spawn is at 0, 0, 0
    Pose2d targetNode = new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90));

    // Driver is pushing forward at 2.0 m/s
    SmartAssistAlign command = new SmartAssistAlign(swerveDrive, () -> 2.0, targetNode);

    command.initialize();

    // Tick the environment for ~0.5 seconds
    for (int i = 0; i < 25; i++) {
      command.execute();
      swerveDrive.periodic();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(0.02);
    }

    Pose2d newPose = swerveDrive.getPose();

    // X should have moved positively (user input)
    assertTrue(newPose.getX() > 0.1, "Should move in positive X due to human input.");

    // Y should have moved positively (auto align)
    assertTrue(
        newPose.getY() > 0.1, "Should automatically strafe leftward (positive Y) toward target.");

    // Theta should have rotated positively (auto align)
    assertTrue(
        newPose.getRotation().getRadians() > 0.1,
        "Should automatically rotate positive toward target theta.");
  }
}
