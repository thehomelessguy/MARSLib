package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShootOnTheMoveCommandTest {

  private SwerveDrive swerveDrive;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().cancelAll();
    com.marslib.simulation.MARSPhysicsWorld.resetInstance();

    GyroIOSim gyroSim = new GyroIOSim();
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());

    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }

    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);
    swerveDrive.resetPose(new Pose2d(0, 0, new Rotation2d()));
  }

  @Test
  public void testStationaryAim() {
    ShootOnTheMoveCommand command = new ShootOnTheMoveCommand(swerveDrive, () -> 0.0, () -> 0.0);

    CommandScheduler.getInstance().schedule(command);

    for (int i = 0; i < 150; i++) {
      DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
    }

    // From (0,0) aiming at BLUE_HUB_POS (approx (0, 5.5)), the expected target angle is 90 degrees.
    Pose2d resultingPose = swerveDrive.getPose();
    double expectedAngleRaw =
        Math.atan2(
            Constants.FieldConstants.BLUE_HUB_POS.getY() - resultingPose.getY(),
            Constants.FieldConstants.BLUE_HUB_POS.getX() - resultingPose.getX());

    double finalDeg = resultingPose.getRotation().getDegrees();
    double expectedDeg = Math.toDegrees(expectedAngleRaw);

    // Assert that the physical bot successfully pivoted its heading statically.
    assertTrue(
        Math.abs(finalDeg - expectedDeg) < 5.0,
        "Physical robot failed to pivot to static target heading. Final: "
            + finalDeg
            + ", Expected: "
            + expectedDeg);
  }

  @Test
  public void testMovingAimCalculatesVirtualTarget() {
    // If the robot is translating laterally along X (Wait, Y gives tangency? It evaluates based on
    // BLUE_HUB_POS)
    // BLUE_HUB_POS is purely positive Y across the field if we start at 0,0.
    // Let's command the robot to drive explicitly along purely X visually (strafe).
    ShootOnTheMoveCommand command = new ShootOnTheMoveCommand(swerveDrive, () -> 2.0, () -> 0.0);

    CommandScheduler.getInstance().schedule(command);

    for (int i = 0; i < 150; i++) {
      DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
    }

    Pose2d resultingPose = swerveDrive.getPose();

    // Robot physically moved!
    assertTrue(
        resultingPose.getX() > 1.0,
        "Physical Robot failed to translate continuously via joysticks in physical testing!");

    double staticAngleRaw =
        Math.atan2(
            Constants.FieldConstants.BLUE_HUB_POS.getY() - resultingPose.getY(),
            Constants.FieldConstants.BLUE_HUB_POS.getX() - resultingPose.getX());

    double expectedStaticDeg = Math.toDegrees(staticAngleRaw);
    double actualDeg = resultingPose.getRotation().getDegrees();

    // Since the robot is translating horizontally, the "aim point" virtual target inherently
    // compensates by aiming ahead of the target, meaning the angles MUST physically diverge from
    // static analysis!
    assertTrue(
        Math.abs(actualDeg - expectedStaticDeg) > 0.5,
        "Robot failed to apply dynamic kinematic offset! Actual: "
            + actualDeg
            + ", Static: "
            + expectedStaticDeg);
  }
}
