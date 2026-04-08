package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.testing.MARSTestHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RotaryMechanismIOSimTest {

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.tearDown();
  }

  @Test
  public void testPidConvergence() {
    // 100:1 ratio, 0.1 moment of inertia
    RotaryMechanismIOSim sim = new RotaryMechanismIOSim("ArmSim", 100.0, 0.1, 0.5);
    RotaryMechanismIOInputsAutoLogged inputs = new RotaryMechanismIOInputsAutoLogged();

    // Command arm to 1.0 radian
    sim.setClosedLoopPosition(1.0, 0.0);

    // Step simulation for 4 seconds (200 ticks)
    for (int i = 0; i < 200; i++) {
      sim.updateInputs(inputs);
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    sim.updateInputs(inputs);

    // Verify it settled near the target
    assertEquals(
        1.0,
        inputs.positionRad,
        0.1,
        "IOSim should converge on PID reference via force translation");
    assertTrue(
        inputs.appliedVolts > 0.0 || inputs.velocityRadPerSec < 0.1, "Should have stabilized");
  }
}
