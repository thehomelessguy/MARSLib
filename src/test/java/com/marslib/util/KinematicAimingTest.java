package com.marslib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marslib.mechanisms.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.Test;

public class KinematicAimingTest {

  @Test
  public void testStationaryAiming() {
    Pose2d currentPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
    Pose2d targetPose =
        new Pose2d(new Translation2d(5, 0), new Rotation2d()); // Target 5m directly ahead
    ChassisSpeeds stationarySpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    double projectileSpeed = 10.0; // 10 m/s

    Pose2d virtualAimPoint =
        KinematicAiming.calculateVirtualAimPoint(
            currentPose, stationarySpeeds, targetPose, projectileSpeed);

    // If robot is stationary, virtual aim point should perfectly mirror the real target
    assertEquals(
        5.0, virtualAimPoint.getX(), 0.01, "Stationary X aim point should equal physical X.");
    assertEquals(
        0.0, virtualAimPoint.getY(), 0.01, "Stationary Y aim point should equal physical Y.");
  }

  @Test
  public void testMovingAiming() {
    Pose2d currentPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
    // Target is exactly 10m away on X-axis
    Pose2d targetPose = new Pose2d(new Translation2d(10, 0), new Rotation2d());
    // Robot moving sideways along Y-axis at 2.0 m/s
    ChassisSpeeds movingSpeeds = new ChassisSpeeds(0.0, 2.0, 0.0);
    // Muzzle velocity = 10 m/s.
    // ToF = 10m / 10m/s = 1 second flight time.
    double projectileSpeed = 10.0;

    Pose2d virtualAimPoint =
        KinematicAiming.calculateVirtualAimPoint(
            currentPose, movingSpeeds, targetPose, projectileSpeed);

    // Since the robot is drifting Y at +2 m/s, over 1 second of flight time the bullet shifts +2m
    // in Y.
    // Therefore, the robot needs to aim at a virtual target positioned at (-2m) in Y to cancel out
    // its physical velocity.
    assertEquals(
        10.0,
        virtualAimPoint.getX(),
        0.01,
        "Virtual X aim point should equal physical X since robot vx is 0.");
    assertEquals(
        -2.0,
        virtualAimPoint.getY(),
        0.01,
        "Virtual Y aim point should be offset -2m backwards to compensate for +2m/s translation speed.");
  }
}
