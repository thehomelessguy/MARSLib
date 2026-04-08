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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSAlignmentCommandTest {

  private SwerveDrive swerveDrive;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    // Required to configure WPI standard HAL hooks for integrated physical Simulation tests
    // com.marslib.simulation.
    // Construct genuine simulation architectures entirely decoupled from Mockito wrappers
    GyroIOSim gyroSim = new GyroIOSim();
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());

    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }

    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);

    // Hard-set robot origin
    swerveDrive.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Test
  public void testCommandDrivesTowardsTargetPhysically() {
    Pose2d targetPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90));
    MARSAlignmentCommand command =
        new MARSAlignmentCommand(
            swerveDrive,
            () -> targetPose,
            5.0,
            5.0,
            3.0,
            3.0,
            Math.PI * 2,
            Math.PI * 4,
            0.05,
            0.05);

    // Start physical controller pipeline
    CommandScheduler.getInstance().schedule(command);

    // The robot starts at (0,0,0).
    // Stepping the CommandScheduler physically invokes the Holonomic PathFollower,
    // translating outputs into Volts, driving the TalonFX simulations, stepping the Dyn4j bodies,
    // reading Wheel slip back into the estimators, and updating Odometry. Total "Digital Twin"
    // pipeline!

    for (int i = 0; i < 150; i++) {
      // Step WPILib HAL timings manually!
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
      CommandScheduler.getInstance().run();
      com.marslib.simulation.MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
    }

    Pose2d resultingPose = swerveDrive.getPose();

    System.out.println("Resulting Pose After Physics Ticks: " + resultingPose);

    // We expect the robot to have physically traveled significantly towards (2, 2)
    // and rotated towards 90 degrees after 3 seconds of simulation time (150 ticks).
    assertTrue(
        resultingPose.getX() > 0.4,
        "Robot failed to traverse physically along X axis due to physics friction constraint or lack of command");
    assertTrue(resultingPose.getY() > 0.4, "Robot failed to traverse physically along Y axis");

    // Command shouldn't be fully finished due to standard path deceleration constraints over 1s.
    assertFalse(command.isFinished(), "Tolerance not met yet physically");
  }

  @Test
  public void testCommandFinishesWhenPhysicallyAligned() {
    Pose2d targetPose = new Pose2d(0.001, 0.001, Rotation2d.fromDegrees(0));
    swerveDrive.resetPose(targetPose);

    MARSAlignmentCommand command =
        new MARSAlignmentCommand(
            swerveDrive,
            () -> targetPose,
            5.0,
            5.0,
            3.0,
            3.0,
            Math.PI * 2,
            Math.PI * 4,
            0.05,
            0.05);
    CommandScheduler.getInstance().schedule(command);

    CommandScheduler.getInstance().run();

    assertTrue(
        command.isFinished(),
        "Should cleanly terminate natively if within PID tolerance limits from start");
  }
}
