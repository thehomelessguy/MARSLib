package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSShooterTest {

  private FlywheelIOSim io;
  private MARSShooter shooter;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    // Reset hardware and physics state before test
    // Shooter uses standard flywheel simulation dynamics (Kraken X60, 1:1 reduction, 0.002 KgM^2)
    io = new FlywheelIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());
    shooter = new MARSShooter(io, powerManager);
  }

  @Test
  public void testManualControlsVerifyPhysicalVoltage() {
    var inputs = new FlywheelIOInputsAutoLogged();

    // Start at expected rest
    shooter.periodic();
    io.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts, 0.1);

    // Apply strict voltage and advance hardware logic
    shooter.setVoltage(10.0);
    shooter.periodic();
    io.updateInputs(inputs);
    assertEquals(10.0, inputs.appliedVolts, 0.1);
  }

  @Test
  public void testClosedLoopVelocityTargeting() {
    var inputs = new FlywheelIOInputsAutoLogged();

    // Apply a target velocity mapping
    shooter.setClosedLoopVelocity(500.0);
    shooter.periodic();
    io.updateInputs(inputs);

    // Ensure target mapped to states
    assertEquals(500.0, inputs.targetVelocityRadPerSec, 0.1);

    // The PID voltage + FF should be non-zero
    assertTrue(inputs.appliedVolts > 0.0);
  }

  @Test
  public void testCommandLifecyclePhysicallyAffectsSubsystem() {
    var inputs = new FlywheelIOInputsAutoLogged();
    var runCommand = shooter.spinUpCommand();

    // Ensure properly registered to subsystem
    assertTrue(runCommand.getRequirements().contains(shooter));

    runCommand.initialize();

    // Fast forward command active state
    for (int i = 0; i < 5; i++) {
      runCommand.execute();
      shooter.periodic();
      io.updateInputs(inputs);
    }

    // Verify target velocity is applied
    assertEquals(400.0, inputs.targetVelocityRadPerSec, 0.1);

    // After spinning up, the wheel should have gained some velocity physically
    assertTrue(inputs.velocityRadPerSec > 0.0);
  }
}
