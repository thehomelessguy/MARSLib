package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants;
import org.junit.jupiter.api.Test;

public class SwerveKinematicsTest {

  @Test
  public void testForwardKinematics() {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_LOCATIONS);

    ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0); // 2 m/s forward
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    // Front Left module
    assertEquals(2.0, states[0].speedMetersPerSecond, 0.001);
    assertEquals(0.0, states[0].angle.getDegrees(), 0.001);

    // Back Right module
    assertEquals(2.0, states[3].speedMetersPerSecond, 0.001);
    assertEquals(0.0, states[3].angle.getDegrees(), 0.001);
  }

  @Test
  public void testRotationalAndTranslationKinematicsIterative() {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_LOCATIONS);

    // Target compound movement: 1.0 m/s Forward, 0.5 m/s Left, Pi rad/s CCW
    ChassisSpeeds targetSpeeds = new ChassisSpeeds(1.0, 0.5, Math.PI);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

    // Feed the discrete module states back into the kinematics to see if it reproduces the exact
    // target speeds
    ChassisSpeeds actualSpeeds = kinematics.toChassisSpeeds(states);

    assertEquals(
        1.0,
        actualSpeeds.vxMetersPerSecond,
        0.01,
        "Forward velocity should translate seamlessly back and forth");
    assertEquals(
        0.5,
        actualSpeeds.vyMetersPerSecond,
        0.01,
        "Strafe velocity should translate seamlessly back and forth");
    assertEquals(
        Math.PI,
        actualSpeeds.omegaRadiansPerSecond,
        0.01,
        "Rotational velocity should translate seamlessly back and forth");
  }
}
