package com.marslib.integration;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.RotaryMechanismIOSim;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.MARSCowl;
import frc.robot.subsystems.MARSIntakePivot;
import frc.robot.subsystems.MARSShooter;
import frc.robot.subsystems.MARSSuperstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotLifecycleTest {

  private SwerveDrive swerveDrive;
  private MARSCowl cowl;
  private MARSIntakePivot intakePivot;
  private MARSShooter floorIntake;
  private MARSShooter shooter;
  private MARSShooter feeder;
  private MARSSuperstructure superstructure;
  private MARSPowerManager powerManager;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();

    DriverStationSim.setAutonomous(true);
    // Construct all subsystems — same wiring as RobotContainer SIM mode
    powerManager = new MARSPowerManager(new PowerIOSim());
    GyroIOSim gyroSim = new GyroIOSim();

    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }

    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);
    swerveDrive.resetPose(new Pose2d(2, 2, new edu.wpi.first.math.geometry.Rotation2d(0)));

    cowl = new MARSCowl(new RotaryMechanismIOSim("Cowl", 50.0, 0.5, 0.5), powerManager);
    intakePivot =
        new MARSIntakePivot(new RotaryMechanismIOSim("IntakePivot", 50.0, 0.5, 0.5), powerManager);
    floorIntake =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);
    shooter =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);
    feeder =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);

    java.util.function.Supplier<java.util.Optional<edu.wpi.first.math.geometry.Translation2d>>
        distSupplier = () -> java.util.Optional.empty(); // Fixed dist
    superstructure =
        new MARSSuperstructure(
            cowl, intakePivot, floorIntake, shooter, feeder, swerveDrive::getPose, distSupplier);
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
  }

  @Test
  public void testFullLifecycleDoesNotCrash() {
    // PHASE 1: Validate all subsystems constructed successfully
    assertNotNull(swerveDrive.getPose(), "SwerveDrive pose null after construction");
    assertEquals(
        MARSSuperstructure.SuperstructureState.STOWED,
        superstructure.getCurrentState(),
        "Superstructure should start STOWED");

    // PHASE 2: Simulate 1.5s of autonomous — all subsystems periodic() running concurrently
    for (int i = 0; i < 150; i++) {
      final int tick = i;
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(0.02);
          },
          "Crash during autonomous at tick " + tick);
    }

    // PHASE 3: Switch to teleop
    DriverStationSim.setAutonomous(false);
    // Run 50 ticks of teleop idle (0.5s)
    for (int i = 0; i < 50; i++) {
      final int tick = i;
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(0.02);
          },
          "Crash during teleop idle at tick " + tick);
    }

    // PHASE 4: Command SCORE and run for 3 seconds (300 ticks)
    CommandScheduler.getInstance()
        .schedule(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE));
    CommandScheduler.getInstance().run();

    assertEquals(
        MARSSuperstructure.SuperstructureState.SCORE,
        superstructure.getCurrentState(),
        "Superstructure should transition to SCORE");

    for (int i = 0; i < 300; i++) {
      final int tick = i;
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(0.02);
          },
          "Crash during SCORE at tick " + tick);
    }

    // PHASE 5: Command STOW and settle for 2 seconds (200 ticks)
    CommandScheduler.getInstance()
        .schedule(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));
    CommandScheduler.getInstance().run();

    assertEquals(
        MARSSuperstructure.SuperstructureState.STOWED,
        superstructure.getCurrentState(),
        "Superstructure should transition back to STOWED");

    for (int i = 0; i < 200; i++) {
      final int tick = i;
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(0.02);
          },
          "Crash during STOW settle at tick " + tick);
    }

    // PHASE 6: Final assertions — the robot survived the full lifecycle
    assertNotNull(swerveDrive.getPose(), "Pose estimator null after lifecycle");
    assertEquals(
        MARSSuperstructure.SuperstructureState.STOWED,
        superstructure.getCurrentState(),
        "Should be STOWED at end");
  }
}
