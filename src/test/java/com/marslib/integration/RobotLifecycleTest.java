package com.marslib.integration;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSIntake;
import com.marslib.mechanisms.MARSShooter;
import com.marslib.mechanisms.MARSSuperstructure;
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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Full robot lifecycle integration test. Constructs all subsystems in SIM mode and exercises the
 * auto → teleop → scoring → stow lifecycle through the physics engine.
 *
 * <p>This catches interaction bugs between subsystems that unit tests miss — the kind of failures
 * that appear at competition when multiple subsystems run concurrently.
 */
public class RobotLifecycleTest {

  private SwerveDrive swerveDrive;
  private MARSElevator elevator;
  private MARSArm arm;
  private MARSIntake intake;
  private MARSShooter shooter;
  private MARSSuperstructure superstructure;
  private MARSPowerManager powerManager;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();

    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();

    // Construct all subsystems — same wiring as RobotContainer SIM mode
    powerManager = new MARSPowerManager(new PowerIOSim());
    GyroIOSim gyroSim = new GyroIOSim();

    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }

    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);
    swerveDrive.resetPose(new Pose2d(2, 2, new edu.wpi.first.math.geometry.Rotation2d(0)));

    elevator =
        new MARSElevator(
            new LinearMechanismIOSim(
                "Elevator",
                ElevatorConstants.GEAR_RATIO,
                ElevatorConstants.SPOOL_DIAMETER_METERS,
                ElevatorConstants.SIM_MASS_KG),
            powerManager);
    arm =
        new MARSArm(
            new RotaryMechanismIOSim(
                "Arm", ArmConstants.GEAR_RATIO, ArmConstants.SIM_MOI, ArmConstants.SIM_LENGTH),
            powerManager);
    intake =
        new MARSIntake(
            new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1.0, 0.01));
    shooter =
        new MARSShooter(
            new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(2), 1.0, 0.05));

    superstructure = new MARSSuperstructure(elevator, arm, intake, shooter, swerveDrive::getPose);
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.tearDown();
  }

  /**
   * Exercises the complete match lifecycle: construct → auto → teleop → score → stow. If any
   * subsystem interaction causes an exception, this test catches it.
   */
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
            SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
          },
          "Crash during autonomous at tick " + tick);
    }

    // PHASE 3: Switch to teleop
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    // Run 50 ticks of teleop idle (0.5s)
    for (int i = 0; i < 50; i++) {
      final int tick = i;
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
          },
          "Crash during teleop idle at tick " + tick);
    }

    // PHASE 4: Command SCORE_HIGH and run for 3 seconds (300 ticks)
    CommandScheduler.getInstance()
        .schedule(
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE_HIGH));
    CommandScheduler.getInstance().run();

    assertEquals(
        MARSSuperstructure.SuperstructureState.SCORE_HIGH,
        superstructure.getCurrentState(),
        "Superstructure should transition to SCORE_HIGH");

    for (int i = 0; i < 300; i++) {
      final int tick = i;
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
          },
          "Crash during SCORE_HIGH at tick " + tick);
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
            SimHooks.stepTiming(Constants.LOOP_PERIOD_SECS);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(Constants.LOOP_PERIOD_SECS);
          },
          "Crash during STOW settle at tick " + tick);
    }

    // PHASE 6: Final assertions — the robot survived the full lifecycle
    assertNotNull(swerveDrive.getPose(), "Pose estimator null after lifecycle");
    assertEquals(
        MARSSuperstructure.SuperstructureState.STOWED,
        superstructure.getCurrentState(),
        "Should be STOWED at end");

    // Elevator should have returned close to 0
    assertTrue(
        elevator.getPositionMeters() < 0.1,
        "Elevator should have settled near 0m after STOW, was " + elevator.getPositionMeters());
  }
}
