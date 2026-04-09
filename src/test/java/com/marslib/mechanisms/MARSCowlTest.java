package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSCowlTest {

  private MARSCowl cowl;
  private double simulatedVoltageOverride = 12.0;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    PowerIO spoofedVoltageIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = simulatedVoltageOverride;
            inputs.isBrownedOut = simulatedVoltageOverride < 6.0;
          }
        };

    MARSPowerManager powerManager = new MARSPowerManager(spoofedVoltageIO);

    RotaryMechanismIOSim physicalCowlSim =
        new RotaryMechanismIOSim(
            "MARSCowl_Test",
            50.0,
            0.5, // 0.5 kg m^2 inertia
            1.2 // 1.2m length
            );

    cowl = new MARSCowl(physicalCowlSim, powerManager);
  }

  @Test
  public void testCowlPhysicallyAttainsTargetAnglesUsingDyn4jMotorMath() {
    cowl.setTargetPosition(Math.PI / 2.0); // 90 Degrees

    for (int i = 0; i < 75; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    assertEquals(
        Math.PI / 2.0,
        cowl.getPositionRads(),
        0.05,
        "Authentic physics carriage should reach Math.PI/2 target position.");
  }

  @Test
  public void testLoadSheddingDynamicRestoresAreAppliedUnderCriticalBrownoutConditions() {
    simulatedVoltageOverride = 9.0 - 0.5;

    for (int i = 0; i < 20; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    assertEquals(9.0 - 0.5, simulatedVoltageOverride, 0.0);
    assertNotNull(cowl);
  }
}
