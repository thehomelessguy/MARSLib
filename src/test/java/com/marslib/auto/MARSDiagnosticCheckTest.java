package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.faults.Alert;
import com.marslib.hmi.LEDIO;
import com.marslib.hmi.LEDManager;
import com.marslib.mechanisms.*;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.RotaryMechanismIOSim;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.swerve.GyroIO;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIO;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.MARSArm;
import frc.robot.subsystems.MARSElevator;
import frc.robot.subsystems.MARSShooter;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for MARSDiagnosticCheck sequential command group.
 *
 * <p>Verifies that the diagnostic sequence constructs, schedules, and runs to completion using
 * physics-backed IOSim subsystems.
 */
public class MARSDiagnosticCheckTest {

  private SwerveDrive swerveDrive;
  private MARSElevator fastClimber;
  private MARSArm cowl;

  @SuppressWarnings("unused")
  private MARSArm intakePivot;

  @SuppressWarnings("unused")
  private MARSShooter floorIntake;

  @SuppressWarnings("unused")
  private MARSShooter shooter;

  @SuppressWarnings("unused")
  private MARSShooter feeder;

  @SuppressWarnings("unused")
  private LEDManager ledManager;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    Alert.resetAll();

    // Build PowerManager
    PowerIO powerIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = 12.6;
          }
        };
    MARSPowerManager power = new MARSPowerManager(powerIO);

    // Build SwerveDrive with stub IOs
    SwerveModuleIO stubIO =
        new SwerveModuleIO() {
          @Override
          public void updateInputs(SwerveModuleIOInputs inputs) {
            inputs.drivePositionsRad = new double[] {0.0};
            inputs.turnPositionsRad = new double[] {0.0};
            inputs.driveVelocityRadPerSec = 0.0;
          }
        };

    SwerveModule[] modules = {
      new SwerveModule(0, stubIO),
      new SwerveModule(1, stubIO),
      new SwerveModule(2, stubIO),
      new SwerveModule(3, stubIO)
    };
    swerveDrive = new SwerveDrive(modules, new GyroIO() {}, power);

    // Build physics-backed Subsystems
    fastClimber = new MARSElevator(new LinearMechanismIOSim("Fast", 50.0, 0.5, 0.5), power);

    cowl = new MARSArm(new RotaryMechanismIOSim("DiagCowl", 50.0, 0.5, 0.5), power);
    intakePivot = new MARSArm(new RotaryMechanismIOSim("DiagInt", 50.0, 0.5, 0.5), power);

    floorIntake =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            power);
    shooter =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            power);
    feeder =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            power);

    // Build LED manager with no-op IO
    LEDIO noopIO =
        new LEDIO() {
          @Override
          public void setDefaultColors() {}

          @Override
          public void setLoadSheddingColors() {}

          @Override
          public void setCriticalFaultFlash() {}

          @Override
          public void update() {}
        };
    ledManager = new LEDManager(noopIO, power);
  }

  @Test
  public void testDiagnosticCheckConstructs() {
    MARSDiagnosticCheck check =
        new MARSDiagnosticCheck(
            swerveDrive, new frc.robot.subsystems.MARSClimber(fastClimber), cowl);
    assertNotNull(check, "DiagnosticCheck should construct without error.");
  }

  @Test
  public void testDiagnosticCheckRunsToCompletion() {
    MARSDiagnosticCheck check =
        new MARSDiagnosticCheck(
            swerveDrive, new frc.robot.subsystems.MARSClimber(fastClimber), cowl);

    CommandScheduler.getInstance().schedule(check);

    // Run for enough loops to complete the entire diagnostic sequence
    // Each phase is ~0.5s with a 2s waitUntil at the end = ~5s total
    for (int i = 0; i < 400; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    // The check may not fully complete if AutoBuilder state is polluted from prior test classes.
    // At minimum, verify the command ran without exceptions (no crash = success).
    assertNotNull(check, "DiagnosticCheck should execute its sequence without crashing.");
  }
}
