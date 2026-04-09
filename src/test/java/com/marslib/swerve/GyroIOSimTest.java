package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.testing.MARSTestHarness;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class GyroIOSimTest {

  private GyroIOSim gyroSim;
  private GyroIOInputsAutoLogged inputs;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    gyroSim = new GyroIOSim();
    inputs = new GyroIOInputsAutoLogged();
  }

  @Test
  public void testInitialStateIsZero() {
    gyroSim.updateInputs(inputs);

    assertTrue(inputs.connected, "Simulated gyro should always report connected.");
    assertEquals(0.0, inputs.yawPositionRad, "Initial position should be 0.");
    assertEquals(0.0, inputs.yawVelocityRadPerSec, "Initial velocity should be 0.");
  }

  @Test
  public void testUpdatesFromPhysicsWorld() {
    // GyroIOSim relies on the simulation ticking to update.
    // It hooks into physics via its internal mechanics.
    // Given we are testing GyroIOSim in isolation without full SwerveDrive,
    // we can only ensure it doesn't crash when running independently.

    // Simulate physics loop
    com.marslib.simulation.MARSPhysicsWorld.getInstance().update(0.02);

    assertDoesNotThrow(() -> gyroSim.updateInputs(inputs));
  }
}
