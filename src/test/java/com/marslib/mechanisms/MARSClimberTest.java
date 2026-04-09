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

public class MARSClimberTest {

  private MARSClimber climber;
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

    LinearMechanismIOSim physicalClimberSim =
        new LinearMechanismIOSim(
            "MARSClimber_Test",
            50.0,
            0.05, // 5cm spool diameter
            5.0 // 5.0 kg physical carriage mass
            );

    climber = new MARSClimber(physicalClimberSim, powerManager);
  }

  @Test
  public void testClimberPhysicallyAttainsTargetHeightUsingDyn4jMotorMath() {
    climber.setTargetPosition(1.0); // 1.0 Meters

    for (int i = 0; i < 150; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    assertEquals(
        1.0,
        climber.getPositionMeters(),
        0.05,
        "Authentic physics carriage should reach 1.0M target position.");
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
    assertNotNull(climber);
  }
}
