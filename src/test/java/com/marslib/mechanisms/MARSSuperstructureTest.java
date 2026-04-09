package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSSuperstructureTest {

  private MARSCowl cowl;
  private MARSIntakePivot intakePivot;
  private MARSShooter floorIntake;
  private MARSShooter shooter;
  private MARSShooter feeder;
  private MARSSuperstructure superstructure;

  private double simulatedVoltageOverride = 12.0;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    PowerIO spoofedVoltageIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIO.PowerIOInputs inputs) {
            inputs.voltage = simulatedVoltageOverride;
            inputs.isBrownedOut = simulatedVoltageOverride < 6.0;
          }
        };

    MARSPowerManager powerManager = new MARSPowerManager(spoofedVoltageIO);

    RotaryMechanismIOSim cowlSim = new RotaryMechanismIOSim("Cowl", 50.0, 0.5, 0.5);
    cowl = new MARSCowl(cowlSim, powerManager);

    RotaryMechanismIOSim intakePivotSim = new RotaryMechanismIOSim("IntakePivot", 50.0, 0.5, 0.5);
    intakePivot = new MARSIntakePivot(intakePivotSim, powerManager);

    FlywheelIOSim physicalShooterSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    shooter = new MARSShooter(physicalShooterSim, powerManager);

    FlywheelIOSim physicalFloorSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    floorIntake = new MARSShooter(physicalFloorSim, powerManager);

    FlywheelIOSim physicalFeederSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    feeder = new MARSShooter(physicalFeederSim, powerManager);

    Supplier<Pose2d> mockSupplier = () -> new Pose2d();
    java.util.function.Supplier<java.util.Optional<edu.wpi.first.math.geometry.Translation2d>>
        distSupplier = () -> java.util.Optional.empty(); // Fixed dist

    superstructure =
        new MARSSuperstructure(
            cowl, intakePivot, floorIntake, shooter, feeder, mockSupplier, distSupplier);
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
  }

  @Test
  public void testScoreDumpingCommand() {
    // Command SCORE phase — legal from ANY
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE).initialize();

    for (int i = 0; i < 50; i++) {
      superstructure.periodic();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }
  }

  @Test
  public void testUnjamState() {
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.UNJAM).initialize();
    for (int i = 0; i < 5; i++) {
      superstructure.periodic();
    }
    assertEquals(MARSSuperstructure.SuperstructureState.UNJAM, superstructure.getCurrentState());
  }

  @Test
  public void testStateMachineTicksAndTransitionCount() {
    // Run a few ticks in STOWED
    for (int i = 0; i < 5; i++) {
      superstructure.periodic();
    }
    assertEquals(5, superstructure.getStateMachine().getTicksInCurrentState());
    assertEquals(0, superstructure.getStateMachine().getTotalTransitionCount());

    // Transition to SCORE
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE).initialize();
    superstructure.periodic();
    assertEquals(1, superstructure.getStateMachine().getTicksInCurrentState());
    assertEquals(1, superstructure.getStateMachine().getTotalTransitionCount());
  }

  @Test
  public void testIntakeToStowedTransition() {
    superstructure
        .setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_DOWN)
        .initialize();
    superstructure
        .setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING)
        .initialize();

    // Simulate INTAKE phase (some ticks)
    for (int i = 0; i < 5; i++) {
      superstructure.periodic();
    }
    assertEquals(
        MARSSuperstructure.SuperstructureState.INTAKE_RUNNING, superstructure.getCurrentState());

    // Switch to STOWED (must go through INTAKE_DOWN)
    superstructure
        .setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_DOWN)
        .initialize();
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED).initialize();
    superstructure.periodic();
    assertEquals(MARSSuperstructure.SuperstructureState.STOWED, superstructure.getCurrentState());
  }

  @Test
  public void testFlywheelFrictionSpindown() {
    // Assert flywheels properly spin down on STOW
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE).initialize();
    for (int i = 0; i < 50; i++) {
      superstructure.periodic();
      DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    // Record current flywheel velocity
    double highRpm = shooter.getVelocityRadPerSec() * 60.0 / (Math.PI * 2.0);
    assertTrue(highRpm > 1000, "Shooter should rev up in SCORE state");

    // STOW
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED).initialize();
    for (int i = 0; i < 50; i++) {
      superstructure.periodic();
      DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    double lowRpm = shooter.getVelocityRadPerSec() * 60.0 / (Math.PI * 2.0);
    assertTrue(lowRpm < highRpm, "Shooter should spin down in STOW state due to friction model");
  }
}
