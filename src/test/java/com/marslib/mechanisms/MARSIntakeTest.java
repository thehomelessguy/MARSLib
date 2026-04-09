package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSIntakeTest {

  private FlywheelIOSim io;
  private MARSIntake intake;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    // Reset hardware and physics state before test
    // Intake uses standard flywheel simulation dynamics (Kraken X60, 2:1 reduction, 0.005 KgM^2)
    io = new FlywheelIOSim(DCMotor.getKrakenX60Foc(1), 2.0, 0.005);
    MARSPowerManager powerManager = new MARSPowerManager(new PowerIOSim());
    intake = new MARSIntake(io, powerManager);
  }

  @Test
  public void testManualControlsVerifyPhysicalVoltage() {
    var inputs = new FlywheelIOInputsAutoLogged();

    // Start at expected rest
    intake.periodic();
    io.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts, 0.1);

    // Apply strict voltage and advance hardware logic
    intake.setVoltage(12.0);
    intake.periodic();
    io.updateInputs(inputs);
    assertEquals(12.0, inputs.appliedVolts, 0.1);

    // Halt logic
    intake.stop();
    intake.periodic();
    io.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts, 0.1);
  }

  @Test
  public void testCommandLifecyclePhysicallyAffectsSubsystem() {
    var inputs = new FlywheelIOInputsAutoLogged();
    var runCommand = intake.runIntakeCommand();

    // Ensure properly registered to subsystem
    assertTrue(runCommand.getRequirements().contains(intake));

    runCommand.initialize();

    // Fast forward command active state
    for (int i = 0; i < 5; i++) {
      runCommand.execute();
      intake.periodic();
      io.updateInputs(inputs);
    }

    // Verify voltage applied properly
    assertEquals(12.0, inputs.appliedVolts, 0.1);

    // Simulate scheduling interrupt
    runCommand.end(false);
    intake.periodic();
    io.updateInputs(inputs);

    assertEquals(0.0, inputs.appliedVolts, 0.1);

    // Run reverse command similarly
    var reverseCommand = intake.runReverseCommand();
    reverseCommand.initialize();
    for (int i = 0; i < 5; i++) {
      reverseCommand.execute();
      intake.periodic();
      io.updateInputs(inputs);
    }
    assertEquals(-12.0, inputs.appliedVolts, 0.1);
  }
}
