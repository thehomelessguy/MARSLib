package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSIntakeTest {

  private FlywheelIOSim io;
  private MARSIntake intake;

  @BeforeEach
  public void setUp() {
    // Reset hardware and physics state before test
    CommandScheduler.getInstance().cancelAll();
    MARSPhysicsWorld.getInstance().resetInstance();

    // Intake uses standard flywheel simulation dynamics (Kraken X60, 2:1 reduction, 0.005 KgM^2)
    io = new FlywheelIOSim(DCMotor.getKrakenX60Foc(1), 2.0, 0.005);
    intake = new MARSIntake(io);
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
